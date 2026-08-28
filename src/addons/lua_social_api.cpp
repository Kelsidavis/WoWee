// lua_social_api.cpp - Chat, guild, friends, ignore, gossip, party management, and emotes Lua API bindings.
// Extracted from lua_engine.cpp as part of §5.1 (Tame LuaEngine).
#include <iterator>
#include <vector>
#include "addons/lua_api_helpers.hpp"
#include "game/reputation_standing.hpp"
#include "rendering/animation_controller.hpp"

namespace wowee::addons {

// The languages a character can speak: their racial one, plus the faction
// tongue every member of that faction is taught. Human and Orc are the two
// where those are the same, so they know exactly one.
//
// Returned as (name, id) pairs rather than read from Languages.dbc: the wire
// ids are fixed across every expansion this client speaks, and the dbc adds
// nothing but a localised name we do not have a table for.
struct LanguageEntry { const char* name; int id; };

// The right each guild control permission checkbox stands for, in the panel's
// order rather than the mask's.
//
// The two orders are not the same and must not be confused: the panel lists
// Promote and Demote fifth and sixth while their bits are 0x80 and 0x100, above
// Invite's 0x10 and Remove's 0x20 - so walking the mask in order would tick the
// wrong four boxes. The names come from GUILDCONTROL_OPTION1..17, which is what
// GuildControlPopupFrame_OnLoad labels each checkbox with; the bits from
// AzerothCore's GuildRankRights.
//
// Seventeen entries, of which the fourteenth has no checkbox - its option
// string exists but the frame was never given the button, and FrameXML's update
// loop skips the gap with a comment saying the flag is deprecated. The slot is
// filled anyway because everything after it is found by counting: drop it and
// the two guild bank rights land on the wrong boxes.
//
// One table for the getter and the setter both. They are inverses over the same
// order, so two copies of it is two chances to renumber only one.
static constexpr uint32_t kGuildRankFlagBits[] = {
    0x00000001u,  // 1  Guildchat Listen
    0x00000002u,  // 2  Guildchat Speak
    0x00000004u,  // 3  Officerchat Listen
    0x00000008u,  // 4  Officerchat Speak
    0x00000080u,  // 5  Promote
    0x00000100u,  // 6  Demote
    0x00000010u,  // 7  Invite Member
    0x00000020u,  // 8  Remove Member
    0x00001000u,  // 9  Set MOTD
    0x00002000u,  // 10 Edit Public Note
    0x00004000u,  // 11 View Officer Note
    0x00008000u,  // 12 Edit Officer Note
    0x00010000u,  // 13 Modify Guild Info
    0u,           // 14 deprecated, no checkbox
    0x00040000u,  // 15 Repair - GR_RIGHT_WITHDRAW_REPAIR
    0x00080000u,  // 16 Gold - GR_RIGHT_WITHDRAW_GOLD
    0x00100000u,  // 17 Create Guild Event
};
static constexpr size_t kGuildRankFlagCount = std::size(kGuildRankFlagBits);

static void collectKnownLanguages(uint8_t raceId, std::vector<LanguageEntry>& out) {
    const bool horde = (raceId == 2 || raceId == 5 || raceId == 6 ||
                        raceId == 8 || raceId == 10);
    // Racial language first - that is the order the dropdown shows them in.
    switch (raceId) {
        case 1:  out.push_back({"Common", 7});      break;
        case 2:  out.push_back({"Orcish", 1});      break;
        case 3:  out.push_back({"Dwarvish", 6});    break;
        case 4:  out.push_back({"Darnassian", 2});  break;
        case 5:  out.push_back({"Gutterspeak", 33}); break;
        case 6:  out.push_back({"Taurahe", 3});     break;
        case 7:  out.push_back({"Gnomish", 13});    break;
        case 8:  out.push_back({"Troll", 14});      break;
        case 10: out.push_back({"Thalassian", 10}); break;
        case 11: out.push_back({"Draenei", 35});    break;
        default: break;
    }
    const LanguageEntry faction = horde ? LanguageEntry{"Orcish", 1}
                                        : LanguageEntry{"Common", 7};
    // Skipped when it is the racial one, so Human does not list Common twice.
    if (out.empty() || out[0].id != faction.id) out.push_back(faction);
}

/// Whether the player's guild rank holds one right. Shared by the nine Can*
/// queries, which differ only in which bit they ask about.
static int luaGuildRight(lua_State* L, uint32_t bit) {
    auto* gh = getGameHandler(L);
    lua_pushboolean(L, (gh && (gh->getPlayerGuildRankRights() & bit)) ? 1 : 0);
    return 1;
}

static int lua_GetNumLanguages(lua_State* L) {
    auto* gh = getGameHandler(L);
    std::vector<LanguageEntry> langs;
    if (gh) collectKnownLanguages(gh->getPlayerRace(), langs);
    if (langs.empty()) langs.push_back({"Common", 7});
    lua_pushnumber(L, static_cast<double>(langs.size()));
    return 1;
}

static int lua_GetLanguageByIndex(lua_State* L) {
    auto* gh = getGameHandler(L);
    std::vector<LanguageEntry> langs;
    if (gh) collectKnownLanguages(gh->getPlayerRace(), langs);
    if (langs.empty()) langs.push_back({"Common", 7});

    const int idx = static_cast<int>(luaL_optnumber(L, 1, 1));
    if (idx < 1 || idx > static_cast<int>(langs.size())) return 0;
    lua_pushstring(L, langs[static_cast<size_t>(idx) - 1].name);
    lua_pushnumber(L, langs[static_cast<size_t>(idx) - 1].id);
    return 2;
}

static int lua_SendChatMessage(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) return 0;
    const char* msg = luaL_checkstring(L, 1);
    const char* chatType = luaL_optstring(L, 2, "SAY");
    // language arg (3) ignored - server determines language
    const char* target = luaL_optstring(L, 4, "");

    std::string typeStr(chatType);
    for (char& c : typeStr) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));

    // Eight of the thirteen were mapped and the rest fell to the SAY this is
    // initialised with, which is the worst possible default: a message meant
    // for a channel or a raid warning went to everyone standing nearby, and
    // nothing about it looked like a failure.
    //
    // CHANNEL is the one that mattered most. Every numbered channel - General,
    // Trade, LookingForGroup - sends through here, so with FrameXML drawing
    // the chat every channel line was said out loud instead.
    game::ChatType ct = game::ChatType::SAY;
    bool known = true;
    if (typeStr == "SAY")            ct = game::ChatType::SAY;
    else if (typeStr == "YELL")      ct = game::ChatType::YELL;
    else if (typeStr == "PARTY")     ct = game::ChatType::PARTY;
    else if (typeStr == "GUILD")     ct = game::ChatType::GUILD;
    else if (typeStr == "OFFICER")   ct = game::ChatType::OFFICER;
    else if (typeStr == "RAID")      ct = game::ChatType::RAID;
    else if (typeStr == "WHISPER")   ct = game::ChatType::WHISPER;
    else if (typeStr == "BATTLEGROUND") ct = game::ChatType::BATTLEGROUND;
    else if (typeStr == "CHANNEL")   ct = game::ChatType::CHANNEL;
    else if (typeStr == "RAID_WARNING") ct = game::ChatType::RAID_WARNING;
    // The custom emote, /e and /me. A different thing from DoEmote: this one
    // carries the player's own words rather than an EmotesText row.
    else if (typeStr == "EMOTE")     ct = game::ChatType::EMOTE;
    // The messages sent to whoever whispers while away. Their own chat types
    // on the wire, and said aloud if they are not.
    else if (typeStr == "AFK")       ct = game::ChatType::AFK;
    else if (typeStr == "DND")       ct = game::ChatType::DND;
    else known = false;
    if (!known) {
        LOG_WARNING("SendChatMessage: unknown chat type '", typeStr,
                    "', not sending - SAY would say it out loud");
        return 0;
    }

    std::string targetStr(target && *target ? target : "");
    // A channel is named on the wire and numbered in the interface.
    // ChatEdit_ParseText stores what GetChannelName answered - an index into
    // the joined list - as the edit box's channelTarget and hands that back
    // here, so a bare number has to be turned into the name before it is sent.
    if (ct == game::ChatType::CHANNEL && lua_isnumber(L, 4)) {
        const int index = static_cast<int>(lua_tonumber(L, 4));
        const auto& joined = gh->getJoinedChannels();
        if (index >= 1 && index <= static_cast<int>(joined.size())) {
            targetStr = joined[static_cast<size_t>(index) - 1];
        } else {
            LOG_WARNING("SendChatMessage: channel index ", index,
                        " is outside the ", joined.size(), " joined, not sending");
            return 0;
        }
    }
    gh->sendChatMessage(ct, msg, targetStr);
    return 0;
}

// SendAddonMessage(prefix, text, chatType, target) - send addon message
static int lua_SendAddonMessage(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) return 0;
    const char* prefix = luaL_checkstring(L, 1);
    const char* text = luaL_checkstring(L, 2);
    const char* chatType = luaL_optstring(L, 3, "PARTY");
    const char* target = luaL_optstring(L, 4, "");

    // Build addon message: prefix + TAB + text, send via the appropriate channel
    std::string typeStr(chatType);
    for (char& c : typeStr) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));

    game::ChatType ct = game::ChatType::PARTY;
    if (typeStr == "PARTY")           ct = game::ChatType::PARTY;
    else if (typeStr == "RAID")       ct = game::ChatType::RAID;
    else if (typeStr == "GUILD")      ct = game::ChatType::GUILD;
    else if (typeStr == "OFFICER")    ct = game::ChatType::OFFICER;
    else if (typeStr == "BATTLEGROUND") ct = game::ChatType::BATTLEGROUND;
    else if (typeStr == "WHISPER")    ct = game::ChatType::WHISPER;

    // Encode as prefix\ttext (WoW addon message format)
    std::string encoded = std::string(prefix) + "\t" + text;
    std::string targetStr(target && *target ? target : "");
    gh->sendAddonMessage(ct, encoded, targetStr);
    return 0;
}

// RegisterAddonMessagePrefix(prefix) - register prefix for receiving addon messages
static int lua_RegisterAddonMessagePrefix(lua_State* L) {
    const char* prefix = luaL_checkstring(L, 1);
    // Store in a global Lua table for filtering
    lua_getglobal(L, "__WoweeAddonPrefixes");
    if (lua_isnil(L, -1)) {
        lua_pop(L, 1);
        lua_newtable(L);
        lua_pushvalue(L, -1);
        lua_setglobal(L, "__WoweeAddonPrefixes");
    }
    lua_pushboolean(L, 1);
    lua_setfield(L, -2, prefix);
    lua_pop(L, 1);
    lua_pushboolean(L, 1); // success
    return 1;
}

// IsAddonMessagePrefixRegistered(prefix) → boolean
static int lua_IsAddonMessagePrefixRegistered(lua_State* L) {
    const char* prefix = luaL_checkstring(L, 1);
    lua_getglobal(L, "__WoweeAddonPrefixes");
    if (lua_istable(L, -1)) {
        lua_getfield(L, -1, prefix);
        lua_pushboolean(L, lua_toboolean(L, -1));
        return 1;
    }
    lua_pushboolean(L, 0);
    return 1;
}

static int lua_GetNumFriends(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { lua_pushnumber(L, 0); lua_pushnumber(L, 0); return 2; }
    // Two values. FrameXML reads the second and adds it to another count on
    // the same line - local _, numWoWOnline = GetNumFriends() - so returning
    // only the total leaves that arithmetic against nil.
    int count = 0, online = 0;
    for (const auto& c : gh->getContacts()) {
        if (!c.isFriend()) continue;
        ++count;
        if (c.status != 0) ++online;   // 0 is offline; 1/2/3 are on, AFK, DND
    }
    lua_pushnumber(L, count);
    lua_pushnumber(L, online);
    return 2;
}

// GetFriendInfo(index or name) → name, level, class, area, connected, status, note
//
// A name as well as a row, because both are passed. The friends list asks by
// row; the right-click menu's Set Note stores the name in FriendsFrame.NotesID
// and the note dialog reads it back through here - and an index-only reading
// through luaL_checknumber raises on a name rather than answering it, which
// took the dialog down as it opened.
static int lua_GetFriendInfo(lua_State* L) {
    auto* gh = getGameHandler(L);
    const bool byName = lua_isstring(L, 1) && !lua_isnumber(L, 1);
    const std::string wanted = byName ? lua_tostring(L, 1) : std::string();
    const int index = byName ? 0 : static_cast<int>(luaL_optnumber(L, 1, 0));
    if (!gh || (!byName && index < 1)) {
        return luaReturnNil(L);
    }
    int found = 0;
    for (const auto& c : gh->getContacts()) {
        if (!c.isFriend()) continue;
        if (byName ? (c.name == wanted) : (++found == index)) {
            lua_pushstring(L, c.name.c_str());      // 1: name
            lua_pushnumber(L, c.level);              // 2: level

            lua_pushstring(L, c.classId < 12 ? kLuaClasses[c.classId] : "Unknown"); // 3: class
            std::string area;
            if (c.areaId != 0) area = gh->getWhoAreaName(c.areaId);
            lua_pushstring(L, area.c_str());         // 4: area
            lua_pushboolean(L, c.isOnline());        // 5: connected
            // The bits, not the number: an away friend is online|away, which
            // is 3, and comparing that for equality named them busy instead.
            // friendsframe tests these against CHAT_FLAG_AFK and CHAT_FLAG_DND,
            // which are these strings.
            constexpr uint8_t kAway = 0x02, kBusy = 0x04;
            lua_pushstring(L, (c.status & kAway) ? "<AFK>"
                            : ((c.status & kBusy) ? "<DND>" : ""));  // 6: status
            lua_pushstring(L, c.note.c_str());       // 7: note
            return 7;
        }
    }
    lua_pushnil(L);
    return 1;
}

// --- Guild API ---

/// The guild roster as the panel is currently asking to see it: offline members
/// dropped if the tick is clear, and ordered by whichever column was last
/// clicked. Everything that takes a roster row index reads it from here.
///
/// The roster arrives in the server's order and used to be handed over in it,
/// with SortGuildRoster and SetGuildRosterShowOffline both doing nothing. That
/// is the whole of the guild panel's sorting and filtering - FrameXML keeps no
/// order of its own, it re-reads the API and redraws - so every column header
/// was a button that played a click and changed nothing, and the "show offline"
/// tick could be cleared with the offline members still listed beneath it.
///
/// Pointers into the roster, which is owned by the handler and outlives the
/// call; the view is rebuilt per call rather than cached, because the roster is
/// replaced wholesale whenever SMSG_GUILD_ROSTER arrives and a cache would go
/// stale exactly when the panel redraws.
namespace {
/// The direction argument these targeting commands take: Bindings.xml gives
/// each of them two keys, one plain and one with a 1 for backwards.
inline bool wantsReverseArg(lua_State* L) {
    return lua_toboolean(L, 1) != 0 || (lua_isnumber(L, 1) && lua_tonumber(L, 1) != 0);
}
}  // namespace

namespace {

/// Turn every addon this client knows about on or off.
///
/// The list is the registry table GetAddOnInfo reads, so the two cannot come to
/// disagree about which addons exist.
int setAllAddOnsEnabled(lua_State* L, bool enabled) {
    auto* svc = getLuaServices(L);
    if (!svc || !svc->setAddOnEnabled) return 0;
    lua_getfield(L, LUA_REGISTRYINDEX, "wowee_addon_info");
    if (!lua_istable(L, -1)) { lua_pop(L, 1); return 0; }
    const int count = static_cast<int>(lua_objlen(L, -1));
    for (int i = 1; i <= count; ++i) {
        lua_rawgeti(L, -1, i);
        if (lua_istable(L, -1)) {
            lua_getfield(L, -1, "name");
            const char* name = lua_tostring(L, -1);
            if (name && *name) svc->setAddOnEnabled(name, enabled);
            lua_pop(L, 1);
        }
        lua_pop(L, 1);
    }
    lua_pop(L, 1);
    return 0;
}


/// Whether an npc a dialog is waiting on is still close enough to talk to.
///
/// Five yards is AzerothCore's INTERACTION_DISTANCE, which is the figure the
/// server itself refuses a request beyond - so a dialog that stays open past
/// it is offering a button that cannot work.
///
/// An npc with no entity here answers *yes*, and the distinction matters: not
/// knowing where someone is is not the same as knowing they are far away. Both
/// callers close a dialog on a no, and the note this replaces warned about
/// exactly that - a client that cannot measure the distance and answers no
/// shuts the dialog the instant it opens. Closing on ignorance would bring
/// that back for any npc briefly missing from the entity list.
bool npcWithinInteractRange(game::GameHandler* gh, uint64_t npcGuid) {
    if (!gh || npcGuid == 0) return true;
    auto npc = gh->getEntityManager().getEntity(npcGuid);
    auto player = gh->getEntityManager().getEntity(gh->getPlayerGuid());
    if (!npc || !player) return true;
    const float dx = player->getX() - npc->getX();
    const float dy = player->getY() - npc->getY();
    const float dz = player->getZ() - npc->getZ();
    constexpr float kInteractionDistance = 5.0f;
    return dx * dx + dy * dy + dz * dz <= kInteractionDistance * kInteractionDistance;
}

/// Which arena team a one-based index names, or zero for none.
///
/// The interface counts teams 1, 2, 3 for the three bracket sizes and passes
/// that position; the wire wants the team's own id, which is what every arena
/// command is addressed by. The same mapping the roster request uses.
bool& guildShowOffline() { static bool show = true; return show; }
std::string& guildSortField() { static std::string field = "name"; return field; }
bool& guildSortReverse() { static bool rev = false; return rev; }

std::vector<const game::GuildRosterMember*> guildRosterView(game::GameHandler* gh) {
    std::vector<const game::GuildRosterMember*> view;
    if (!gh) return view;
    const auto& roster = gh->getGuildRoster();
    view.reserve(roster.members.size());
    for (const auto& m : roster.members) {
        if (!guildShowOffline() && !m.online) continue;
        view.push_back(&m);
    }
    const std::string& by = guildSortField();
    // Ties break on name so a redraw cannot reorder equal rows under the
    // player's cursor - a level sort over a guild of forty otherwise shuffles
    // every row of the same level each time the roster arrives.
    auto less = [&](const game::GuildRosterMember* a, const game::GuildRosterMember* b) {
        if (by == "level")  { if (a->level != b->level) return a->level < b->level; }
        else if (by == "class")  { if (a->classId != b->classId) return a->classId < b->classId; }
        else if (by == "rank")   { if (a->rankIndex != b->rankIndex) return a->rankIndex < b->rankIndex; }
        else if (by == "note")   { if (a->publicNote != b->publicNote) return a->publicNote < b->publicNote; }
        else if (by == "zone")   {
            const std::string za = a->online ? gh->getWhoAreaName(a->zoneId) : std::string();
            const std::string zb = b->online ? gh->getWhoAreaName(b->zoneId) : std::string();
            if (za != zb) return za < zb;
        } else if (by == "online") {
            // Longest offline last, which is what the "last online" column
            // means; anyone online sorts above everyone who is not.
            if (a->online != b->online) return a->online > b->online;
            if (a->lastOnline != b->lastOnline) return a->lastOnline < b->lastOnline;
        }
        return a->name < b->name;
    };
    std::stable_sort(view.begin(), view.end(), less);
    if (guildSortReverse()) std::reverse(view.begin(), view.end());
    return view;
}

/// The who results in the order the panel last asked for. Same story as the
/// guild roster: the list arrives in the server's order, every column header
/// calls SortWho with its field and nothing else, and FrameXML keeps no order
/// of its own - so the headers and the sort dropdown were both inert.
///
/// "group" is one of the fields the headers send and there is nothing here to
/// answer it with: SMSG_WHO carries no party for the players it lists. It
/// falls through to the name, which is the same order the column had before.
std::string& whoSortField() { static std::string field = "name"; return field; }
bool& whoSortReverse() { static bool rev = false; return rev; }

std::vector<const game::WhoEntry*> whoResultsView(game::GameHandler* gh) {
    std::vector<const game::WhoEntry*> view;
    if (!gh) return view;
    for (const auto& w : gh->getWhoResults()) view.push_back(&w);
    const std::string& by = whoSortField();
    auto less = [&](const game::WhoEntry* a, const game::WhoEntry* b) {
        if (by == "level")      { if (a->level != b->level) return a->level < b->level; }
        else if (by == "class") { if (a->classId != b->classId) return a->classId < b->classId; }
        else if (by == "race")  { if (a->raceId != b->raceId) return a->raceId < b->raceId; }
        else if (by == "guild") { if (a->guildName != b->guildName) return a->guildName < b->guildName; }
        else if (by == "zone")  {
            const std::string za = gh->getWhoAreaName(a->zoneId);
            const std::string zb = gh->getWhoAreaName(b->zoneId);
            if (za != zb) return za < zb;
        }
        return a->name < b->name;
    };
    std::stable_sort(view.begin(), view.end(), less);
    if (whoSortReverse()) std::reverse(view.begin(), view.end());
    return view;
}

}  // namespace

// Declared in lua_api_helpers.hpp: the system API asks the same question and
// had its own copy under another name.
uint32_t arenaTeamIdAtIndex(game::GameHandler* gh, int index) {
    if (!gh || index < 1) return 0;
    const auto& teams = gh->getArenaTeamStats();
    if (index > static_cast<int>(teams.size())) return 0;
    return teams[static_cast<size_t>(index) - 1].teamId;
}

// IsInGuild() → boolean
static int lua_IsInGuild(lua_State* L) {
    auto* gh = getGameHandler(L);
    lua_pushboolean(L, gh && gh->isInGuild());
    return 1;
}

// GetGuildInfo("player") → guildName, guildRankName, guildRankIndex
static int lua_GetGuildInfoFunc(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh || !gh->isInGuild()) { return luaReturnNil(L); }
    lua_pushstring(L, gh->getGuildName().c_str());
    // Get rank name for the player
    const auto& roster = gh->getGuildRoster();
    std::string rankName;
    uint32_t rankIndex = 0;
    for (const auto& m : roster.members) {
        if (m.guid == gh->getPlayerGuid()) {
            rankIndex = m.rankIndex;
            const auto& rankNames = gh->getGuildRankNames();
            if (rankIndex < rankNames.size()) rankName = rankNames[rankIndex];
            break;
        }
    }
    lua_pushstring(L, rankName.c_str());
    lua_pushnumber(L, rankIndex);
    return 3;
}

// GetNumGuildMembers() → totalMembers, onlineMembers
static int lua_GetNumGuildMembers(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { lua_pushnumber(L, 0); lua_pushnumber(L, 0); return 2; }
    // The first count is how many rows there are to draw, so it has to be the
    // filtered one: the panel loops 1..this and reads each index, and counting
    // the whole roster while indexing a filtered view walked off the end of it.
    // The second is how many of the guild are online, filtered or not, which is
    // what the "N of M online" line means.
    const auto view = guildRosterView(gh);
    int online = 0;
    for (const auto& m : gh->getGuildRoster().members)
        if (m.online) online++;
    lua_pushnumber(L, view.size());
    lua_pushnumber(L, online);
    return 2;
}

// GetGuildRosterInfo(index) → name, rank, rankIndex, level, class, zone, note, officerNote, online, status, classId
static int lua_GetGuildRosterInfo(lua_State* L) {
    auto* gh = getGameHandler(L);
    int index = static_cast<int>(luaL_checknumber(L, 1));
    if (!gh || index < 1) { return luaReturnNil(L); }
    const auto view = guildRosterView(gh);
    if (index > static_cast<int>(view.size())) { return luaReturnNil(L); }
    const auto& m = *view[static_cast<size_t>(index) - 1];

    lua_pushstring(L, m.name.c_str());                      // 1: name
    const auto& rankNames = gh->getGuildRankNames();
    lua_pushstring(L, m.rankIndex < rankNames.size()
        ? rankNames[m.rankIndex].c_str() : "");              // 2: rank name
    lua_pushnumber(L, m.rankIndex);                          // 3: rankIndex
    lua_pushnumber(L, m.level);                              // 4: level
    lua_pushstring(L, m.classId < 12 ? kLuaClasses[m.classId] : "Unknown"); // 5: class
    std::string zone;
    if (m.zoneId != 0 && m.online) zone = gh->getWhoAreaName(m.zoneId);
    lua_pushstring(L, zone.c_str());                         // 6: zone
    lua_pushstring(L, m.publicNote.c_str());                 // 7: note
    lua_pushstring(L, m.officerNote.c_str());                // 8: officerNote
    lua_pushboolean(L, m.online);                            // 9: online
    lua_pushnumber(L, 0);                                    // 10: status (0=online, 1=AFK, 2=DND)
    // The uppercase token, not the id. This position is classFileName and it
    // is a key, not a number:
    //
    //     if ( classFileName ) then
    //         classTextColor = RAID_CLASS_COLORS[classFileName];
    //     else
    //         classTextColor = NORMAL_FONT_COLOR;
    //     end
    //     ... buttonText:SetTextColor(classTextColor.r, ...)
    //
    // An id is truthy, so it took the first branch, found no such key, and
    // read .r off a nil one line later - inside GuildStatus_Update, which
    // takes the whole roster down with it. Only on the online branch, so a
    // guild whose members were all offline drew fine and one with anybody on
    // showed nothing, which is what "mostly unpopulated" looks like.
    //
    // The guard is there because the value can be absent. A binding that
    // answers something of the wrong kind defeats it, the same way "UNKNOWN"
    // did in UnitClass and a boolean did in GetChecked.
    // The refusal as well as the answer, from one place: "" is truthy in Lua
    // and would pass the same guard and fail the same lookup as the id did.
    luaPushClassToken(L, m.classId);                          // 11: classFileName
    return 11;
}

// GetGuildRosterMOTD() → motd
static int lua_GetGuildRosterMOTD(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { lua_pushstring(L, ""); return 1; }
    lua_pushstring(L, gh->getGuildRoster().motd.c_str());
    return 1;
}

// GetNumIgnores() → count
static int lua_GetNumIgnores(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { return luaReturnZero(L); }
    int count = 0;
    for (const auto& c : gh->getContacts())
        if (c.isIgnored()) count++;
    lua_pushnumber(L, count);
    return 1;
}

// GetIgnoreName(index) → name
static int lua_GetIgnoreName(lua_State* L) {
    auto* gh = getGameHandler(L);
    int index = static_cast<int>(luaL_checknumber(L, 1));
    if (!gh || index < 1) { return luaReturnNil(L); }
    int found = 0;
    for (const auto& c : gh->getContacts()) {
        if (!c.isIgnored()) continue;
        if (++found == index) {
            lua_pushstring(L, c.name.c_str());
            return 1;
        }
    }
    lua_pushnil(L);
    return 1;
}

// --- Talent API ---

// GetNumTalentTabs() → count (usually 3)

/// Battle.net friends: none, and none online. Two values, because FriendsFrame
/// reads them together and does arithmetic on the second the line it is read -
/// numBNetTotal - numBNetOnline.
// ── Battle.net, which this client has none of ──────────────────────────────
//
// Answering rather than omitting. An unbound name is a raise, and these are
// not all behind BNFeaturesEnabled the way the takeover notes assumed:
// IgnoreList_Update opens with GetNumIgnores, BNGetNumBlocked and
// BNGetNumBlockedToons on three consecutive lines with no guard between them,
// so the ignore tab died on the second of the three. FriendsFrameTooltip_Show
// and FriendsFriendsList_Update are unguarded in the same way.
//
// Zero friends, zero blocks and nil rows is what a client with no Battle.net
// honestly has, and every reader above them treats that as an empty list.
static int lua_BNZero(lua_State* L)  { lua_pushnumber(L, 0.0); return 1; }
static int lua_BNFalse(lua_State* L) { lua_pushboolean(L, 0); return 1; }
static int lua_BNNothing(lua_State* L) { (void)L; return 0; }

/// n nils, for the row readers. Each is asked for a row of a list that is
/// empty, so the count of values matters and none of them does.
template <int N>
static int lua_BNNilRow(lua_State* L) {
    for (int i = 0; i < N; ++i) lua_pushnil(L);
    return N;
}

/// BNGetNumFOF() → total, online. Two, because both are read.
static int lua_BNTwoZeros(lua_State* L) {
    lua_pushnumber(L, 0.0);
    lua_pushnumber(L, 0.0);
    return 2;
}

/// BNGetCustomMessageTable(t) → t, online broadcasts, offline broadcasts.
///
/// The two counts go straight into the scroll height arithmetic, so they have
/// to be numbers rather than nil. The table is handed back as it came: the
/// caller keeps it between calls and replaces it with a fresh one on nil.
static int lua_BNCustomMessageTable(lua_State* L) {
    if (lua_istable(L, 1)) lua_pushvalue(L, 1);
    else                   lua_newtable(L);
    lua_pushnumber(L, 0.0);
    lua_pushnumber(L, 0.0);
    return 3;
}

static int lua_BNGetNumFriends(lua_State* L) {
    lua_pushnumber(L, 0.0);
    lua_pushnumber(L, 0.0);
    return 2;
}


// GetAutoCompleteResults(text, include, exclude, max, cursorPos) → names
//
// What the box under a whisper or a mail address offers as you type. Returns
// the names themselves, one value each, because AutoComplete_UpdateResults
// reads them with select rather than out of a table.
//
// The sources are the ones this client actually knows: the group, the guild
// roster and the friends list. Battle.net has none here, so its flag matches
// nothing rather than pretending.
static int lua_GetAutoCompleteResults(lua_State* L) {
    auto* gh = getGameHandler(L);
    std::string prefix = luaL_optstring(L, 1, "");
    const uint32_t include = static_cast<uint32_t>(luaL_optnumber(L, 2, 0xFFFFFFFFu));
    const uint32_t exclude = static_cast<uint32_t>(luaL_optnumber(L, 3, 0));
    const int wanted = static_cast<int>(luaL_optnumber(L, 4, 5));
    if (!gh || wanted <= 0) return 0;

    constexpr uint32_t kInGroup = 0x01, kInGuild = 0x02, kFriend = 0x04, kOnline = 0x20;

    auto lower = [](std::string v) {
        for (char& c : v) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        return v;
    };
    const std::string want = lower(prefix);

    // One name may be a guildmate and a friend both, so what it is comes first
    // and the flags are merged before anything is decided about it.
    std::map<std::string, uint32_t> byName;
    auto note = [&](const std::string& name, uint32_t flag, bool online) {
        if (name.empty()) return;
        if (!want.empty() && lower(name).compare(0, want.size(), want) != 0) return;
        byName[name] |= flag | (online ? kOnline : 0u);
    };

    for (const auto& mbr : gh->getPartyData().members) note(mbr.name, kInGroup, mbr.isOnline != 0);
    for (const auto& mbr : gh->getGuildRoster().members) note(mbr.name, kInGuild, mbr.online);
    for (const auto& c : gh->getContacts()) {
        if (c.isFriend()) note(c.name, kFriend, c.isOnline());
    }

    // `wanted` comes from the caller and nothing bounds it; the names come
    // from a guild roster, which can be a thousand. Room first - pushing past
    // the stack top corrupts the heap rather than raising.
    if (!lua_checkstack(L, wanted + 1)) return 0;
    int pushed = 0;
    for (const auto& [name, flags] : byName) {
        if (pushed >= wanted) break;
        if ((flags & exclude) != 0) continue;
        if (include != 0 && (flags & include) == 0) continue;
        lua_pushstring(L, name.c_str());
        ++pushed;
    }
    return pushed;
}



/// Which row the friends and ignore lists have selected. The client has no
/// opinion about either; they are what the player last clicked.
static int& selectedFriend() { static int v = 0; return v; }
static int& selectedIgnore() { static int v = 0; return v; }

// SendSystemMessage(text) - put a line in the chat as the client itself would
static int lua_SendSystemMessage(lua_State* L) {
    auto* gh = getGameHandler(L);
    const char* msg = luaL_optstring(L, 1, "");
    if (gh && msg && *msg) gh->addSystemChatMessage(msg);
    return 0;
}

// AddOrRemoveFriend(name, note) / AddOrDelIgnore(name) - the slash commands
//
// /friend and /ignore both toggle: naming someone already on the list takes
// them off it. That is what the "OrRemove" and "OrDel" in the names mean, and
// getting it backwards would remove the friend the player was trying to add.
static int lua_AddOrRemoveFriend(lua_State* L) {
    auto* gh = getGameHandler(L);
    const std::string name = luaL_optstring(L, 1, "");
    const std::string note = luaL_optstring(L, 2, "");
    if (!gh || name.empty()) return 0;
    for (const auto& c : gh->getContacts()) {
        if (c.isFriend() && c.name == name) { gh->removeFriend(name); return 0; }
    }
    gh->addFriend(name, note);
    return 0;
}

static int lua_AddOrDelIgnore(lua_State* L) {
    auto* gh = getGameHandler(L);
    const std::string name = luaL_optstring(L, 1, "");
    if (!gh || name.empty()) return 0;
    for (const auto& c : gh->getContacts()) {
        if (c.isIgnored() && c.name == name) { gh->removeIgnore(name); return 0; }
    }
    gh->addIgnore(name);
    return 0;
}

// --- The confirmations that answer another player ---
//
// Every one of these popups already appears: the client fires
// PARTY_INVITE_REQUEST, GUILD_INVITE_REQUEST, RESURRECT_REQUEST and
// DUEL_REQUESTED, and StaticPopup draws them. None of the buttons did anything,
// because the functions behind them were never bound - so an invitation could
// be seen and not taken.
static int lua_AcceptGroup(lua_State* L) {
    if (auto* gh = getGameHandler(L)) gh->acceptGroupInvite();
    return 0;
}
static int lua_DeclineGroup(lua_State* L) {
    if (auto* gh = getGameHandler(L)) gh->declineGroupInvite();
    return 0;
}
static int lua_AcceptGuild(lua_State* L) {
    if (auto* gh = getGameHandler(L)) gh->acceptGuildInvite();
    return 0;
}
static int lua_DeclineGuild(lua_State* L) {
    if (auto* gh = getGameHandler(L)) gh->declineGuildInvite();
    return 0;
}
static int lua_AcceptResurrect(lua_State* L) {
    // The other end of the same question: whether the popup's Accept button
    // reached this at all. StaticPopup_OnClick decides accept from the
    // button's GetID, so a button answering zero would run OnCancel instead
    // and decline the offer while looking like it accepted.
    LOG_INFO("AcceptResurrect called from the interface");
    if (auto* gh = getGameHandler(L)) gh->acceptResurrect();
    return 0;
}
static int lua_DeclineResurrect(lua_State* L) {
    if (auto* gh = getGameHandler(L)) gh->declineResurrect();
    return 0;
}
static int lua_AcceptDuel(lua_State* L) {
    if (auto* gh = getGameHandler(L)) gh->acceptDuel();
    return 0;
}
// A duel is refused by forfeiting it, which is the only way this client has to
// say no and is what the server reads as a decline before it starts.
static int lua_CancelDuel(lua_State* L) {
    if (auto* gh = getGameHandler(L)) gh->forfeitDuel();
    return 0;
}

// --- Reputation ---
//
// The panel is a list of factions with a bar showing how far through the
// current standing the player is. The bands are the game's own and fixed:
// hated starts at -42000 and exalted at 42000, with the rest between.
namespace {

/// Give or take raid assistant for the group member with this name.
///
/// Names are how the unit popup identifies people here, and the group roster is
/// the only name-to-guid map this client keeps - which is enough, because only
/// a group member can be an assistant.
static void setGroupAssistantByName(lua_State* L, bool apply) {
    auto* gh = getGameHandler(L);
    const char* who = luaL_optstring(L, 1, "");
    if (!gh || !who || !*who) return;
    for (const auto& m : gh->getPartyData().members) {
        if (m.name != who) continue;
        gh->setGroupAssistant(m.guid, apply);
        return;
    }
}


/// Which guild roster row is selected. Panel state with no counterpart in the
/// game, so it lives here - the same shape as selectedFriend and selectedSkill.
///
/// The getter answered a flat zero and the setter forgot, which is what killed
/// note editing: every dialog that edits a note reads the selection to know
/// whose note it is, and index zero is nobody.
int& selectedGuildRosterRow() { static int row = 0; return row; }

/// The name in a guild roster row, or empty. The note calls take an index and
/// the client's own verbs take a name, so this is where the two meet.
std::string guildRosterNameAt(game::GameHandler* gh, int index) {
    if (!gh || index < 1) return {};
    const auto view = guildRosterView(gh);
    if (index > static_cast<int>(view.size())) return {};
    return view[static_cast<size_t>(index) - 1]->name;
}

/// Whether the player has opted out of loot rolls. The server is told and
/// keeps the real state; nothing sends it back, so it is remembered here for
/// the menu that shows a tick beside it.
bool& optOutOfLoot() { static bool out = false; return out; }

/// Which row the panel has selected. The client has no opinion - it is what
/// the player last clicked - so it lives here rather than being invented.
int& selectedFaction() {
    static int selected = 1;
    return selected;
}

}  // namespace

// --- Gossip quest lists ---
//
// Which of an NPC's quests are on offer and which are already taken is decided
// by the icon the server sent, not by anything the client tracks. The values
// are the server's QUEST_STATUS enum and are the same ones quest_handler.cpp
// classifies by - kept in step with it deliberately, since a quest sorted into
// the wrong list is offered twice or not at all.
namespace {

/// Daily quests take a different icon in the list. Protocol-defined, WotLK.
constexpr uint32_t kQuestFlagsDaily = 0x1000;

bool gossipQuestIsAvailable(uint32_t icon)   { return icon == 2 || icon == 7 || icon == 8; }
bool gossipQuestIsCompletable(uint32_t icon) { return icon == 5 || icon == 6 || icon == 10; }
bool gossipQuestIsIncomplete(uint32_t icon)  { return icon == 3 || icon == 4; }
/// Taken already, finished or not - which is one list in the gossip window.
bool gossipQuestIsActive(uint32_t icon) {
    return gossipQuestIsIncomplete(icon) || gossipQuestIsCompletable(icon);
}

/// The i-th quest of one kind, or null past the end. One walk shared by
/// everything that indexes these lists, so they cannot disagree about which
/// quest is second.
const game::GossipQuestItem* gossipQuestAt(lua_State* L, bool available, int index) {
    auto* gh = getGameHandler(L);
    if (!gh || index < 1) return nullptr;
    int seen = 0;
    for (const auto& q : gh->getCurrentGossip().quests) {
        const bool matches = available ? gossipQuestIsAvailable(q.questIcon)
                                       : gossipQuestIsActive(q.questIcon);
        if (matches && ++seen == index) return &q;
    }
    return nullptr;
}

int countGossipQuests(lua_State* L, bool available) {
    auto* gh = getGameHandler(L);
    if (!gh) return 0;
    int n = 0;
    for (const auto& q : gh->getCurrentGossip().quests) {
        if (available ? gossipQuestIsAvailable(q.questIcon)
                      : gossipQuestIsActive(q.questIcon)) ++n;
    }
    return n;
}

int pushGossipQuestTitle(lua_State* L, bool available) {
    const int index = static_cast<int>(luaL_checknumber(L, 1));
    const auto* q = gossipQuestAt(L, available, index);
    lua_pushstring(L, q ? q->title.c_str() : "");
    return 1;
}

/// Index into whichever of the two lists the caller means, then ask for that
/// quest by id - the position in the filtered list is not the position in the
/// packet, so the id is the only thing safe to send.
int selectGossipQuestAt(lua_State* L, bool available) {
    auto* gh = getGameHandler(L);
    const int index = static_cast<int>(luaL_checknumber(L, 1));
    if (!gh || index < 1) return 0;
    int seen = 0;
    for (const auto& q : gh->getCurrentGossip().quests) {
        const bool matches = available ? gossipQuestIsAvailable(q.questIcon)
                                       : gossipQuestIsActive(q.questIcon);
        if (!matches) continue;
        if (++seen != index) continue;
        gh->selectGossipQuest(q.questId);
        return 0;
    }
    return 0;
}

}  // namespace

// --- GM tickets ---
//
// GetGMTicket asks; it does not answer. The reply arrives as UPDATE_TICKET,
// fired from the SMSG_GMTICKET_GETTICKET handler, with no arguments at all when
// there is no ticket - which is how the help frame tells the two apart.

// GetGMTicket() → asks the server what ticket this player has open
static int lua_GetGMTicket(lua_State* L) {
    if (auto* gh = getGameHandler(L)) gh->requestGmTicket();
    return 0;
}

// NewGMTicket(text, needResponse) → opens one
static int lua_NewGMTicket(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) return 0;
    const char* text = luaL_checkstring(L, 1);
    // needResponse is not carried by the packet this client sends, so it is
    // read and dropped rather than quietly changing what is submitted.
    gh->submitGmTicket(text);
    return 0;
}

// DeleteGMTicket() → withdraws it
static int lua_DeleteGMTicket(lua_State* L) {
    if (auto* gh = getGameHandler(L)) gh->deleteGmTicket();
    return 0;
}

// --- Icon pickers ---
//
// The macro frame and the guild bank tab dialog are both grids over the same
// list of icons: how many, then one at a time by index. WoW numbers them from
// one, and the first is the question mark every unset macro shows.

static const std::vector<std::string>* iconList(lua_State* L) {
    auto* services = getLuaServices(L);
    if (!services || !services->listIconTextures) return nullptr;
    return &services->listIconTextures();
}

static int lua_GetNumMacroIcons(lua_State* L) {
    const auto* icons = iconList(L);
    lua_pushnumber(L, icons ? static_cast<double>(icons->size()) : 0.0);
    return 1;
}

// GetMacroIconInfo(index) → a texture path, or nil past the end
static int lua_GetMacroIconInfo(lua_State* L) {
    const auto* icons = iconList(L);
    const int index = static_cast<int>(luaL_checknumber(L, 1));
    if (!icons || index < 1 || index > static_cast<int>(icons->size())) {
        lua_pushnil(L);
        return 1;
    }
    lua_pushstring(L, (*icons)[static_cast<size_t>(index - 1)].c_str());
    return 1;
}

// ── Guild and arena charters ──────────────────────────────────────────
//
// Every one of these was missing while the client underneath had the whole
// flow: it parses the signature list, tracks who has signed, and its own popup
// signs and turns in. SMSG_PETITION_SHOW_SIGNATURES already fires PETITION_SHOW,
// so the interface's frame opened on cue and then raised on the first call it
// made, which is worse than never opening.
//
// This client only ever holds a guild charter - arena charters are bought
// through a registrar it does not implement - so the type is reported as
// "guild" rather than guessed from the signature count. Saying "arena" wrongly
// would relabel the whole frame and ask for a team size that is not there.

/// The petition currently being shown, or nullptr when there is none.
static const game::PetitionInfo* shownPetition(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) return nullptr;
    const auto& info = gh->getPetitionInfo();
    return info.petitionGuid != 0 ? &info : nullptr;
}

// GetPetitionInfo() → type, title, bodyText, maxSignatures, originatorName,
//                     isOriginator, minSignatures
static int lua_GetPetitionInfo(lua_State* L) {
    auto* gh = getGameHandler(L);
    const auto* info = shownPetition(L);
    if (!gh || !info) return 0;
    const bool isOwner = info->ownerGuid == gh->getPlayerGuid();
    const std::string& owner = gh->lookupName(info->ownerGuid);
    // The heading and both confirmations branch on this, so a charter called
    // a guild charter when it is an arena one is wrong twice over. The query
    // reply says which; until it lands, guild is the commoner answer.
    lua_pushstring(L, info->isArena ? "arena" : "guild");
    lua_pushstring(L, info->guildName.c_str());
    lua_pushstring(L, "");                                   // body text: not sent
    lua_pushnumber(L, info->signaturesRequired);
    lua_pushstring(L, owner.empty() ? "Unknown" : owner.c_str());
    lua_pushboolean(L, isOwner ? 1 : 0);
    lua_pushnumber(L, info->signaturesRequired);
    return 7;
}

// CanSignPetition() → whether the Sign button should be live.
//
// The originator cannot sign their own charter, and neither can someone who
// already has. Both are answered from the signature list rather than assumed,
// so the button greys out the moment the server confirms the signature.
static int lua_CanSignPetition(lua_State* L) {
    auto* gh = getGameHandler(L);
    const auto* info = shownPetition(L);
    if (!gh || !info) { lua_pushboolean(L, 0); return 1; }
    const uint64_t me = gh->getPlayerGuid();
    if (info->ownerGuid == me) { lua_pushboolean(L, 0); return 1; }
    for (const auto& sig : info->signatures) {
        if (sig.playerGuid == me) { lua_pushboolean(L, 0); return 1; }
    }
    lua_pushboolean(L, info->signatureCount < info->signaturesRequired ? 1 : 0);
    return 1;
}

static int lua_GetNumPetitionNames(lua_State* L) {
    const auto* info = shownPetition(L);
    lua_pushnumber(L, info ? static_cast<double>(info->signatures.size()) : 0.0);
    return 1;
}

// GetPetitionNameInfo(index) → the signer's name.
//
// Only GUIDs come down the wire, so the name is resolved the same way this
// client's own popup resolves it. A signer who has never been seen has no name
// to give, and the row says so rather than going blank.
static int lua_GetPetitionNameInfo(lua_State* L) {
    auto* gh = getGameHandler(L);
    const auto* info = shownPetition(L);
    const int index = static_cast<int>(luaL_checknumber(L, 1));
    if (!gh || !info || index < 1 || index > static_cast<int>(info->signatures.size())) {
        lua_pushnil(L);
        return 1;
    }
    const std::string& name = gh->lookupName(info->signatures[static_cast<size_t>(index - 1)].playerGuid);
    lua_pushstring(L, name.empty() ? "Unknown" : name.c_str());
    return 1;
}

static int lua_SignPetition(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (const auto* info = shownPetition(L); gh && info) gh->signPetition(info->petitionGuid);
    return 0;
}

// OfferPetition() → hand the charter to the current target to sign.
//
// The interface offers this only to the originator, which matches the server:
// nobody else can present someone else's charter.
static int lua_OfferPetition(lua_State* L) {
    auto* gh = getGameHandler(L);
    const auto* info = shownPetition(L);
    if (!gh || !info) return 0;
    const uint64_t target = gh->getTargetGuid();
    if (target == 0) {
        gh->addUIError("You have no target.");
        return 0;
    }
    gh->offerPetition(info->petitionGuid, target);
    return 0;
}

static int lua_ClosePetition(lua_State* L) {
    if (auto* gh = getGameHandler(L)) gh->clearPetitionSignaturesUI();
    return 0;
}

// GetNumGuildEvents() / GetGuildEventInfo(i) → the guild's event log.
//
// The log arrives as MSG_GUILD_EVENT_LOG_QUERY and is parsed against
// AzerothCore's own writer rather than guessed - see handleGuildEventLog.
//
// GetGuildEventInfo answers type, player1, player2, rank, year, month, day,
// hour. The wire sends an age in seconds, not a date, so the four time fields
// are derived from it the way the interface expects them: year and month count
// backwards from now in whole units, day is days-ago within the month, hour is
// hours-ago within the day. That is what FriendsFrame formats, and it is why a
// log entry reads "3 days ago" rather than a calendar date.
static int lua_GetNumGuildEvents(lua_State* L) {
    auto* gh = getGameHandler(L);
    // Asked for the first time it is wanted, because nothing else asks.
    //
    // The whole chain is here - MSG_GUILD_EVENT_LOG_QUERY goes out,
    // handleGuildEventLog reads the reply and fires GUILD_EVENT_LOG_UPDATE,
    // and friendsframe registers for it - with one link missing at each end.
    // FrameXML would have made the request from ToggleGuildEventLog, and
    // Blizzard commented that line out, so opening the tab asked nobody for
    // anything and drew an empty log.
    //
    // GuildEventLog_Update calls this first, so the tab opening is what sends
    // it: empty log, request, reply, event, and the update runs again with
    // entries. Once per session, so a guild with genuinely no events does not
    // ask again every time the tab is drawn.
    static bool asked = false;
    if (gh && !asked && gh->getGuildEventLog().empty() && gh->isInGuild()) {
        asked = true;
        gh->requestGuildEventLog();
    }
    lua_pushnumber(L, gh ? static_cast<double>(gh->getGuildEventLog().size()) : 0.0);
    return 1;
}

static int lua_GetGuildEventInfo(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int index = static_cast<int>(luaL_checknumber(L, 1));
    if (!gh) return luaReturnNil(L);
    const auto& log = gh->getGuildEventLog();
    if (index < 1 || index > static_cast<int>(log.size())) return luaReturnNil(L);
    const auto& e = log[static_cast<size_t>(index - 1)];

    static const char* kTypes[] = {
        "", "invite", "join", "promote", "demote", "remove", "quit"
    };
    lua_pushstring(L, (e.type < 7) ? kTypes[e.type] : "");
    lua_pushstring(L, gh->lookupName(e.playerGuid).c_str());
    lua_pushstring(L, e.otherGuid ? gh->lookupName(e.otherGuid).c_str() : "");
    lua_pushnumber(L, e.newRank);

    // Whole units of age, largest first, each within its parent.
    const uint32_t secs = e.secondsAgo;
    lua_pushnumber(L, secs / (365u * 86400u));            // years ago
    lua_pushnumber(L, (secs / (30u * 86400u)) % 12u);     // months within the year
    lua_pushnumber(L, (secs / 86400u) % 30u);             // days within the month
    lua_pushnumber(L, (secs / 3600u) % 24u);              // hours within the day
    return 8;
}

namespace {

/// The pet spell a name refers to, or zero. Named rather than numbered because
/// the autocast slash commands pass what the player typed.
uint32_t petSpellByName(game::GameHandler* gh, const char* name) {
    if (!gh || !name || !*name) return 0;
    std::string want(name);
    toLowerInPlace(want);
    for (uint32_t id : gh->getPetSpells()) {
        std::string have = gh->getSpellName(id);
        toLowerInPlace(have);
        if (have == want) return id;
    }
    return 0;
}

/// Put a pet spell's autocast into a definite state.
int setPetAutocastByName(lua_State* L, bool on) {
    auto* gh = getGameHandler(L);
    const uint32_t id = petSpellByName(gh, luaL_optstring(L, 1, ""));
    if (gh && id != 0 && gh->isPetSpellAutocast(id) != on) {
        gh->togglePetSpellAutocast(id);
    }
    return 0;
}

}  // namespace

void registerSocialLuaAPI(lua_State* L) {
    static const struct { const char* name; lua_CFunction func; } api[] = {
                {"BNGetNumFriends",     lua_BNGetNumFriends},
                {"BNConnected",              lua_BNFalse},
                {"BNIsSelf",                 lua_BNFalse},
                {"BNGetMatureLanguageFilter", lua_BNFalse},
                {"BNGetNumBlocked",          lua_BNZero},
                {"BNGetNumBlockedToons",     lua_BNZero},
                {"BNGetNumFriendToons",      lua_BNZero},
                {"BNGetSelectedFriend",      lua_BNZero},
                {"BNGetSelectedBlock",       lua_BNZero},
                {"BNGetSelectedToonBlock",   lua_BNZero},
                {"BNGetNumFOF",              lua_BNTwoZeros},
                {"BNGetBlockedInfo",         lua_BNNilRow<2>},
                {"BNGetBlockedToonInfo",     lua_BNNilRow<2>},
                {"BNGetFOFInfo",             lua_BNNilRow<4>},
                {"BNGetInfo",                lua_BNNilRow<5>},
                {"BNGetFriendToonInfo",      lua_BNNilRow<12>},
                {"BNGetCustomMessageTable",  lua_BNCustomMessageTable},
                // The verbs. Nothing to send them to, and a name that is not
                // there raises where a verb that does nothing simply does
                // nothing - which is the truth for a client with no
                // Battle.net.
                {"BNSetSelectedFriend",      lua_BNNothing},
                {"BNSetSelectedBlock",       lua_BNNothing},
                {"BNSetSelectedToonBlock",   lua_BNNothing},
                {"BNSetBlocked",             lua_BNNothing},
                {"BNSetCustomMessage",       lua_BNNothing},
                {"BNSetFriendNote",          lua_BNNothing},
                {"BNSetAFK",                 lua_BNNothing},
                {"BNSetDND",                 lua_BNNothing},
                {"BNSendFriendInvite",       lua_BNNothing},
                {"BNSendFriendInviteByID",   lua_BNNothing},
                {"BNDeclineFriendInvite",    lua_BNNothing},
                {"BNRemoveFriend",           lua_BNNothing},
                {"BNReportPlayer",           lua_BNNothing},
                {"BNCreateConversation",     lua_BNNothing},
                {"GetGMTicket",         lua_GetGMTicket},
                // UpdateGMTicket(text) - rewrite the ticket already open.
                // Creating one carries the player's position because a new
                // ticket records where it was raised; editing does not move it,
                // and the request is the text alone.
                {"UpdateGMTicket", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* text = luaL_optstring(L, 1, "");
            if (gh && text && *text) gh->updateGmTicket(text);
            return 0;
        }},
                {"GetNumMacroIcons",    lua_GetNumMacroIcons},
                {"GetMacroIconInfo",    lua_GetMacroIconInfo},
                {"GetNumGuildEvents",   lua_GetNumGuildEvents},
                {"GetGuildEventInfo",   lua_GetGuildEventInfo},
                {"GetPetitionInfo",     lua_GetPetitionInfo},
                {"CanSignPetition",     lua_CanSignPetition},
                {"GetNumPetitionNames", lua_GetNumPetitionNames},
                {"GetPetitionNameInfo", lua_GetPetitionNameInfo},
                {"SignPetition",        lua_SignPetition},
                {"OfferPetition",       lua_OfferPetition},
                {"ClosePetition",       lua_ClosePetition},
                // GMResponseResolve() - the "close this reply" button on a GM
                // ticket's answer. An empty packet; the server replies by
                // deciding whether to ask for a survey. Unbound, the button
                // raised and the reply could not be dismissed.
                {"GMResponseResolve", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L)) gh->resolveGMResponse();
            return 0;
        }},
                // GetPetitionItemInfo(index) → name, texture, price.
                //
                // The arena registrar prices whichever tab is open, and its
                // three tabs are the three charters the vendor listed - the
                // two, three and five person teams. It calls this on opening a
                // tab and waits for a callback when the answer is nothing,
                // which is what happens before the item query comes back.
                {"GetPetitionItemInfo", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int index = static_cast<int>(luaL_optnumber(L, 1, 1)) - 1;
            uint32_t itemId = 0, displayId = 0, cost = 0;
            if (!gh || !gh->getPetitionCharter(index, itemId, displayId, cost)) return 0;
            const auto* info = gh->getItemInfo(itemId);
            if (!info || !info->valid) { gh->ensureItemInfo(itemId); return 0; }
            lua_pushstring(L, info->name.c_str());
            lua_pushstring(L, gh->getItemIconPath(
                displayId ? displayId : info->displayInfoId).c_str());
            lua_pushnumber(L, cost);
            return 3;
        }},
                // TurnInArenaPetition(teamSize, r,g,b, icon, ...) - the banner
                // designer's Accept.
                //
                // Every argument after the first is the tabard being drawn on
                // screen, and the server reads none of them:
                // HandleTurnInPetitionOpcode takes the petition's guid and
                // nothing else. So the charter whose window is open is the
                // whole of the request, and the colours are a client-side
                // preview of a banner the server assigns itself.
                {"TurnInArenaPetition", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L))
                gh->turnInPetition(gh->getPetitionInfo().petitionGuid);
            return 0;
        }},
                // --- The guild registrar ---
                //
                // Reachable since PETITION_SHOWLIST started firing an event,
                // which is what made these three matter: the panel opened and
                // its Purchase button raised.
                //
                // BuyGuildCharter takes the name typed into the frame; the
                // vendor is whichever one this client is standing at.
                {"BuyGuildCharter", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* name = luaL_optstring(L, 1, "");
            if (gh && name && *name) gh->buyPetition(gh->getPetitionNpcGuid(), name);
            return 0;
        }},
                // BuyPetition(index, name) - the arena registrar's Purchase.
                // The index is the tab that was open, which is the arena slot
                // plus one, and it is the only thing telling the server a
                // five-person charter from a two-person one.
                {"BuyPetition", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int index = static_cast<int>(luaL_optnumber(L, 1, 1));
            const char* name = luaL_optstring(L, 2, "");
            if (gh && name && *name && index >= 1)
                gh->buyPetition(gh->getPetitionNpcGuid(), name,
                                static_cast<uint32_t>(index));
            return 0;
        }},
                {"CloseGuildRegistrar", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L)) gh->closePetitionVendor();
            return 0;
        }},
                // What the charter costs, in copper, as the offer said. This
                // was noise until the showlist was read properly: the count is
                // one byte and there is an index in front of every charter,
                // and reading neither put the cost three bytes out.
                {"GetGuildCharterCost", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushnumber(L, gh ? gh->getPetitionCost() : 0);
            return 1;
        }},
                // RenamePetition(name) - the Accept on the rename dialog,
                // which is how a charter gets the guild or team name it will
                // be turned in under. The petition being renamed is the one
                // whose window is open.
                {"RenamePetition", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* newName = luaL_optstring(L, 1, "");
            if (gh && newName && *newName) {
                gh->renamePetition(gh->getPetitionInfo().petitionGuid, newName);
            }
            return 0;
        }},
                // --- Arena team invitations, and disbanding one ---
                //
                // All three sit on a static popup's buttons, so each was a
                // raise on the click rather than on the window: the invite
                // arrived, the dialog appeared, and answering it took the
                // interface down. Accept and decline carry no payload; the
                // server answers from the invite it is already holding.
                {"AcceptArenaTeam", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L)) gh->acceptArenaTeamInvite();
            return 0;
        }},
                {"DeclineArenaTeam", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L)) gh->declineArenaTeamInvite();
            return 0;
        }},
                {"ArenaTeamDisband", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L))
                gh->disbandArenaTeam(static_cast<uint32_t>(luaL_optnumber(L, 1, 0)));
            return 0;
        }},
                // DeclineInvite(name) - the group invite popup's Decline.
                // The same refusal DeclineGroup sends; the name is which
                // invite the dialog was showing, and this client only ever
                // holds one.
                {"DeclineInvite",       lua_DeclineGroup},
                // ComplainInboxItem(index) - reporting a mail as spam.
                //
                // Type 0 is mail, and the three values after the sender are
                // the ones the server logs and ignores: a zero, the mail's own
                // id, and another zero.
                {"ComplainInboxItem", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int index = static_cast<int>(luaL_optnumber(L, 1, 0)) - 1;
            if (!gh) return 0;
            const auto& inbox = gh->getMailInbox();
            if (index < 0 || index >= static_cast<int>(inbox.size())) return 0;
            gh->reportMailSpam(inbox[index].senderGuid, inbox[index].messageId);
            return 0;
        }},
                // ComplainChat(lineID) - reporting a chat line as spam.
                //
                // The line id is a chat frame's own numbering for a message it
                // is still holding, and this client keeps no such number: the
                // report would have to name the sender, the language, the
                // channel and the text, and none of that can be recovered from
                // an id nothing recorded. Sending it with a zero guid would
                // report nobody, which is worse than not sending - so this
                // takes the click and does nothing, and the popup closes.
                // Report a chat line as spam. The unit menu offers it on a
                // right-click of a name in chat and hands back the line id from
                // the player link; the popup that confirms it calls this with
                // the same id.
                //
                // The report itself was already built - CMSG_COMPLAIN, which
                // this server has a real handler for - and reachable only from
                // this client's own window, because every chat line carried the
                // id zero and CanComplainChat is asked about that id before the
                // entry is shown at all.
                {"ComplainChat", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const uint32_t lineId = static_cast<uint32_t>(luaL_optnumber(L, 1, 0));
            if (!gh || lineId == 0) return 0;
            const uint64_t guid = gh->chatLineSender(lineId);
            if (guid != 0) gh->reportPlayer(guid, "");
            return 0;
        }},
                // The petition *vendor* - the guild master or arena
                // registrar offering a charter - as opposed to the charter
                // itself, which ClosePetition above shuts. Both windows read
                // different state, so closing one has to close the other.
                {"ClosePetitionVendor", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L)) gh->closePetitionVendor();
            return 0;
        }},
                // HasFilledPetition() gates the arena registrar's turn-in
                // buttons. Whether a charter has *enough* signatures is not
                // something this client is told - SMSG_PETITION_SHOW_SIGNATURES
                // carries a count, but only for the charter whose window is
                // open, and the required number lives in the server's own
                // config. What can be answered is whether the player is
                // carrying a charter at all, which is the question that
                // decides whether those buttons could ever do anything: an
                // unfilled one is refused by the server with its own error,
                // where hiding the button leaves no way to find out.
                {"HasFilledPetition", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            // Guild Charter, then the 2v2, 3v3 and 5v5 arena charters. Fixed
            // ids in every expansion that has them.
            static const uint32_t kCharters[] = {5863, 23560, 23561, 23562};
            bool carrying = false;
            if (gh) {
                const auto& inv = gh->getInventory();
                for (uint32_t id : kCharters) {
                    for (int i = 0; i < inv.getBackpackSize() && !carrying; ++i)
                        carrying = inv.getBackpackSlot(i).item.itemId == id;
                    for (int b = 0; b < game::Inventory::NUM_BAG_SLOTS && !carrying; ++b)
                        for (int sl = 0; sl < inv.getBagSize(b) && !carrying; ++sl)
                            carrying = inv.getBagSlot(b, sl).item.itemId == id;
                    if (carrying) break;
                }
            }
            lua_pushboolean(L, carrying ? 1 : 0);
            return 1;
        }},
                // CloseTabardCreation() - the tabard designer shutting itself
                // when ShowUIPanel could not put it on screen.
                //
                // Nothing to close. This client has no tabard window and no
                // tabard session: MSG_TABARDVENDOR_ACTIVATE is not handled and
                // OPEN_TABARD_FRAME is never fired, so TabardFrame is only
                // reachable through its own close path - which is this, and
                // which raised. A no-op is the honest shape of that until the
                // vendor side exists; answering it wrongly would be inventing
                // a session to end.
                {"CloseTabardCreation", [](lua_State* L) -> int {
            (void)L;
            return 0;
        }},
                // The guild bank tab dialog picks from the same icons under a
                // name of its own.
                {"GetNumMacroItemIcons", lua_GetNumMacroIcons},
                {"GetMacroItemIconInfo", lua_GetMacroIconInfo},
                {"NewGMTicket",         lua_NewGMTicket},
                {"DeleteGMTicket",      lua_DeleteGMTicket},
                // The long-form guild information, which is a different field
                // from the message of the day and has its own opcode. The
                // roster already carries the current text.
                {"GetGuildInfoText", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) return luaReturnNil(L);
            lua_pushstring(L, gh->getGuildRoster().guildInfo.c_str());
            return 1;
        }},
                {"SetGuildInfoText", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* text = luaL_optstring(L, 1, "");
            if (gh && text) gh->setGuildInfoText(text);
            return 0;
        }},
                // Whether the roster lists members who are offline. A display
                // choice the client does not keep, and the roster it is given
                // holds everyone either way - so the list shows them, which is
                // what answering true says.
                {"GetGuildRosterShowOffline", [](lua_State* L) -> int {
            lua_pushboolean(L, guildShowOffline() ? 1 : 0);
            return 1;
        }},
                {"SetGuildRosterShowOffline", [](lua_State* L) -> int {
            guildShowOffline() = lua_toboolean(L, 1) != 0;
            return 0;
        }},
                // SortWho(field) - name, zone, guild, race, level, class or
                // group, from a column header or the sort dropdown. Same
                // contract as SortGuildRoster: the same field twice reverses.
                {"SortWho", [](lua_State* L) -> int {
            const std::string field = luaL_optstring(L, 1, "name");
            if (field == whoSortField()) whoSortReverse() = !whoSortReverse();
            else { whoSortField() = field; whoSortReverse() = false; }
            return 0;
        }},
                // Voice again: nothing can be muted, so nothing is added to the
                // list and there is no list to update.
                // The last two the social frame reaches for, both belonging to
                // systems this server has no counterpart to: Battle.net
                // presence, and picking which voice channel is live.
                {"GetAutoCompletePresenceID", [](lua_State* L) -> int { return luaReturnNil(L); }},
                {"SetActiveVoiceChannelBySessionID", [](lua_State* L) -> int { (void)L; return 0; }},
                {"AddMute",         [](lua_State* L) -> int { (void)L; return 0; }},
                {"MutedList_Update", [](lua_State* L) -> int { (void)L; return 0; }},

                // ---- Guild rank permissions ----
                //
                // SMSG_GUILD_ROSTER carries a rights bitmask per rank and names
                // which rank each member holds. Both were already parsed and
                // stored; nothing read them, so every one of these answered
                // through the missing-API fallback and the guild panel offered
                // actions the player may not have.
                //
                // The bits are the server's own: invite 0x10, remove 0x20,
                // promote 0x80, demote 0x100, set MOTD 0x1000, edit public note
                // 0x2000, view officer note 0x4000, edit officer note 0x8000.
                {"CanGuildInvite",      [](lua_State* L) -> int { return luaGuildRight(L, 0x00000010u); }},
                {"CanGuildRemove",      [](lua_State* L) -> int { return luaGuildRight(L, 0x00000020u); }},
                {"CanGuildPromote",     [](lua_State* L) -> int { return luaGuildRight(L, 0x00000080u); }},
                {"CanGuildDemote",      [](lua_State* L) -> int { return luaGuildRight(L, 0x00000100u); }},
                {"CanEditMOTD",         [](lua_State* L) -> int { return luaGuildRight(L, 0x00001000u); }},
                {"CanEditPublicNote",   [](lua_State* L) -> int { return luaGuildRight(L, 0x00002000u); }},
                {"CanViewOfficerNote",  [](lua_State* L) -> int { return luaGuildRight(L, 0x00004000u); }},
                {"CanEditOfficerNote",  [](lua_State* L) -> int { return luaGuildRight(L, 0x00008000u); }},
                {"CanEditGuildInfo",    [](lua_State* L) -> int { return luaGuildRight(L, 0x00010000u); }},

                // GetGuildRosterLastOnline(index) → how long since that member
                // was last seen, as years, months, days, hours.
                {"GetGuildRosterLastOnline", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int idx = static_cast<int>(luaL_optnumber(L, 1, 0));
            if (!gh || idx < 1) return luaReturnNil(L);
            const auto view = guildRosterView(gh);
            if (idx > static_cast<int>(view.size())) return luaReturnNil(L);
            // The roster reports it in days, fractionally.
            const float days = view[static_cast<size_t>(idx) - 1]->lastOnline;
            const int totalDays = static_cast<int>(days);
            lua_pushnumber(L, totalDays / 365);          // years
            lua_pushnumber(L, (totalDays % 365) / 30);   // months
            lua_pushnumber(L, (totalDays % 365) % 30);   // days
            lua_pushnumber(L, static_cast<int>((days - totalDays) * 24.0f)); // hours
            return 4;
        }},
                // GetGuildEventInfo is registered once, further up, with the
                // implementation that reads the log. A second entry here
                // answered nil and, being later, won - so the reader above it
                // could never run and the tab was empty whatever arrived. The
                // note that stood here said the request is never made, which
                // was true of the request and not of the reader.
                // One realm, so everyone is on it.
                {"UnitIsSameServer", [](lua_State* L) -> int { lua_pushboolean(L, 1); return 1; }},
                // Recruit-a-Friend summoning, which needs a linked account.
                {"CanSummonFriend", luaReturnFalse},
                {"SummonFriend",    [](lua_State* L) -> int { (void)L; return 0; }},
                // The voice mute list, which cannot have entries without voice.
                {"GetSelectedMute", [](lua_State* L) -> int { return luaReturnNil(L); }},
                {"SetSelectedMute", [](lua_State* L) -> int { (void)L; return 0; }},
                {"GetMuteName",     [](lua_State* L) -> int { return luaReturnNil(L); }},
                {"DelMute",         [](lua_State* L) -> int { (void)L; return 0; }},
                // GetGuildBankTabPermissions(tab) → canView, canDeposit,
                //   canUpdateText, withdrawPerDay
                //
                // The roster carries these per rank per tab; they were read
                // past to stay aligned with the packet and discarded. The bits
                // are view 0x01, deposit 0x02, update text 0x04.
                {"GetGuildBankTabPermissions", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int tab = static_cast<int>(luaL_optnumber(L, 1, 0));
            if (!gh || tab < 1 || tab > 6) return luaReturnNil(L);
            const auto& roster = gh->getGuildRoster();
            const int sel = gh->getSelectedGuildRank();
            if (sel < 1 || sel > static_cast<int>(roster.ranks.size())) return luaReturnNil(L);
            const auto& rank = roster.ranks[static_cast<size_t>(sel) - 1];
            const uint32_t flags = rank.bankTabRights[static_cast<size_t>(tab) - 1];
            lua_pushboolean(L, (flags & 0x01u) ? 1 : 0);   // 1: canView
            lua_pushboolean(L, (flags & 0x02u) ? 1 : 0);   // 2: canDeposit
            lua_pushboolean(L, (flags & 0x04u) ? 1 : 0);   // 3: canUpdateText
            lua_pushnumber(L, rank.bankTabSlotsPerDay[static_cast<size_t>(tab) - 1]);
            return 4;
        }},
                // GetGuildBankWithdrawLimit() → gold per day for the selected
                // rank. The roster calls it goldLimit and it was already parsed.
                {"GetGuildBankWithdrawLimit",  [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) { lua_pushnumber(L, 0); return 1; }
            const auto& roster = gh->getGuildRoster();
            const int sel = gh->getSelectedGuildRank();
            if (sel < 1 || sel > static_cast<int>(roster.ranks.size())) { lua_pushnumber(L, 0); return 1; }
            lua_pushnumber(L, roster.ranks[static_cast<size_t>(sel) - 1].goldLimit);
            return 1;
        }},
                // The three that stage the bank half of a rank edit. Bits are
                // view 0x01, deposit 0x02, update text 0x04 - the same the
                // roster reports.
                {"SetGuildBankTabPermissions", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int tab  = static_cast<int>(luaL_optnumber(L, 1, 0));
            const int perm = static_cast<int>(luaL_optnumber(L, 2, 0));
            const bool on  = lua_toboolean(L, 3) != 0;
            if (!gh || tab < 1 || tab > 6 || perm < 1 || perm > 3) return 0;
            const uint32_t bit = 1u << (perm - 1);
            auto& p = gh->pendingGuildRankRef();
            if (on) p.tabRights[static_cast<size_t>(tab) - 1] |= bit;
            else    p.tabRights[static_cast<size_t>(tab) - 1] &= ~bit;
            return 0;
        }},
                {"SetGuildBankTabWithdraw", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int tab = static_cast<int>(luaL_optnumber(L, 1, 0));
            const auto n  = static_cast<uint32_t>(luaL_optnumber(L, 2, 0));
            if (!gh || tab < 1 || tab > 6) return 0;
            gh->pendingGuildRankRef().tabSlots[static_cast<size_t>(tab) - 1] = n;
            return 0;
        }},
                {"SetGuildBankWithdrawLimit", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (gh) gh->pendingGuildRankRef().goldLimit =
                static_cast<uint32_t>(luaL_optnumber(L, 1, 0));
            return 0;
        }},

                // ---- Mail: invoices, stationery, and the spam report ----
                //
                // GetInboxInvoiceInfo(id) → invoiceType, itemName, playerName,
                //   bid, buyout, deposit, consignment, moneyDelay, etaHour, etaMin
                //
                // This answered ten nils beside a comment saying nothing here
                // parsed an invoice body. Something did: parseAuctionMailBody
                // has been decoding them all along, and this client's own mail
                // window has been printing the breakdown from it. Handing mail
                // over left that behind, so a sale arrived as a letter with the
                // raw colon-separated body and no figures.
                //
                // Found by asking which parsed fields nothing reads.
                // AuctionMailInvoice::ownerGuidLow was one, and a field nobody
                // reads usually means a branch nobody runs.
                {"GetInboxInvoiceInfo", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int index = static_cast<int>(luaL_optnumber(L, 1, 0));
            if (!gh) {
                for (int i = 0; i < 10; ++i) lua_pushnil(L);
                return 10;
            }
            // Bound after the null check rather than through a conditional
            // with an empty vector on the other side: that ternary yields a
            // prvalue, so it copies the whole inbox on every call in the case
            // that matters and only looks like it binds a reference.
            const auto& inbox = gh->getMailInbox();
            if (index < 1 || index > static_cast<int>(inbox.size())) {
                for (int i = 0; i < 10; ++i) lua_pushnil(L);
                return 10;
            }
            const auto& mail = inbox[static_cast<size_t>(index) - 1];
            game::AuctionMailInvoice invoice;
            if (mail.messageType != 2 ||
                !game::parseAuctionMailBody(mail.body, invoice)) {
                for (int i = 0; i < 10; ++i) lua_pushnil(L);
                return 10;
            }

            // Which side of the sale this letter is. The auction house sends
            // the buyer a "won" mail and the seller a "sold" one, and the two
            // halves of OpenMail_Update draw entirely different panels from it
            // - the buyer's shows what was paid, the seller's the deposit and
            // the house's cut.
            //
            // The subject carries the response code that says which.
            game::AuctionMailSubject subject;
            const bool haveSubject =
                game::parseAuctionMailSubject(mail.subject, subject);
            // A sold-auction invoice names a deposit and a cut; a purchase
            // does not, which is the difference the panel is drawn from.
            const bool seller = invoice.deposit > 0 || invoice.consignment > 0;
            lua_pushstring(L, seller ? "seller" : "buyer");

            const uint32_t entry = haveSubject ? subject.itemEntry : 0;
            const auto* info = entry ? gh->getItemInfo(entry) : nullptr;
            lua_pushstring(L, info ? info->name.c_str() : "");

            // The other party. Their guid low is what the invoice carries, and
            // the name cache is asked rather than the server: an invoice is
            // read long after the sale and the seller may be anywhere.
            std::string other;
            if (invoice.ownerGuidLow != 0) {
                for (const auto& [guid, name] : gh->getPlayerNameCache()) {
                    if (static_cast<uint32_t>(guid & 0xFFFFFFFFu) != invoice.ownerGuidLow) {
                        continue;
                    }
                    other = name;
                    break;
                }
            }
            // A name, even an unknown one: FrameXML draws the whole panel
            // behind `if ( playerName )`, so answering nothing here would keep
            // it shut for the sake of a label.
            lua_pushstring(L, other.empty() ? "Unknown" : other.c_str());

            lua_pushnumber(L, invoice.bid);
            lua_pushnumber(L, invoice.buyout);
            lua_pushnumber(L, invoice.deposit);
            lua_pushnumber(L, invoice.consignment);
            // The hour the money becomes available, which the server does not
            // send. Nil rather than a guess: the panel guards each of these and
            // shows "not yet sent" without them, which is true.
            lua_pushnil(L);
            lua_pushnil(L);
            lua_pushnil(L);
            return 10;
        }},
                // Stationery is the letterhead a mail is written on. The list
                // is empty - GetNumStationeries answers zero from the counting
                // table in lua_engine.cpp - so the picker has no rows and these
                // are the calls around it.
                //
                // Deliberately NOT bound here. That table records every name it
                // answers for under a "count:" prefix so it stays in the
                // missing-API report, and binding it explicitly would take it
                // out - which is the one thing that report exists to prevent.
                // The stationery list is a real feature this client does not
                // have, and it should keep saying so.
                // Stationery is the paper a letter is written on. There is one
                // - the plain parchment every letter uses unless the player
                // bought something else - and answering that there are none
                // left the popup empty with nothing to pick.
                //
                // That is not cosmetic. SendMailFrame_CanSend counts three
                // things before it will enable the Send button, and one of them
                // is that a stationery has been chosen. With an empty list
                // there was no way to choose one, so the button could never be
                // enabled and no letter could be sent at all.
                //
                // GetStationeryInfo(index) → name, texture, cost. A nil cost
                // means free, which is what the frame draws as no price.
                {"GetStationeryInfo", [](lua_State* L) -> int {
                    if (static_cast<int>(luaL_optnumber(L, 1, 0)) != 1) return luaReturnNil(L);
                    lua_pushstring(L, "Default");
                    lua_pushstring(L, "Default");   // STATIONERY_PATH..texture.."1"
                    lua_pushnil(L);                 // free
                    return 3;
                }},
                {"GetSelectedStationeryTexture", [](lua_State* L) -> int {
                    lua_pushstring(L, "Default");
                    return 1;
                }},
                // Which one is chosen is the frame's business - it keeps the
                // index itself and hands it here. Nothing on this side varies
                // with it while there is only one.
                {"SelectStationery", [](lua_State* L) -> int { (void)L; return 0; }},
                // Guarded with `if ( texture )` before being pasted into a
                // path, so nil leaves the default parchment rather than
                // building a texture name out of nothing.
                // Reporting a mail as spam needs a GM channel this client does
                // not have, so no mail can be complained about.
                {"CanComplainInboxItem", luaReturnFalse},
                // Turning a letter into a keepable item. Called from the mail
                // frame's XML rather than its Lua, which is why a scan of the
                // Lua alone never reported it.
                {"TakeInboxTextItem", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const auto id = static_cast<uint32_t>(luaL_optnumber(L, 1, 0));
            if (gh && id) gh->takeInboxTextItem(id);
            return 0;
        }},
                // SetSendMailShowing(showing) - the send-mail tab is open, or
                // is not.
                //
                // This client does hold a draft: the letter's attachments live
                // on its side, and clicking an item in a bag attaches it only
                // while the compose frame is up. Discarding this left that flag
                // false for as long as FrameXML owned the mail window, so a
                // click on a bag item went to the branch below it and used the
                // item instead - the food got eaten rather than posted.
                {"SetSendMailShowing", [](lua_State* L) -> int {
                    auto* gh = getGameHandler(L);
                    if (gh) gh->setMailComposeShowing(lua_toboolean(L, 1) != 0);
                    return 0;
                }},

                // ---- The help frame's GM requests ----
                //
                // There is no GM to reach: tickets are submitted through
                // NewGMTicket, and these three are the paths beside it - asking
                // for a lag report, saying an answer did not help, and polling
                // whether a GM is available. Answered rather than left missing
                // because the help frame calls the last one from its OnLoad.
                // Stuck() - the help frame's "I'm stuck" button.
                //
                // In WoW this casts the Stuck spell, which the server answers
                // by moving the character to a graveyard. Cast rather than
                // stubbed, because the button hides the help frame straight
                // after and a no-op would look like the request was made.
                {"Stuck", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (gh) gh->castSpell(7355);
            return 0;
        }},
                {"GMReportLag", [](lua_State* L) -> int { (void)L; return 0; }},
                {"GMResponseNeedMoreHelp", [](lua_State* L) -> int { (void)L; return 0; }},
                // GetGMStatus() - ask whether the ticket queue is open.
                //
                // A request, not a getter: the answer comes back as
                // UPDATE_GM_STATUS. The help frame calls this from OnShow and
                // waits for the event, so a no-op here meant it waited forever
                // and kept whatever it was last told.
                {"GetGMStatus", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L)) gh->requestGmSystemStatus();
            return 0;
        }},

                // ---- Guild rank editing ----
                // GuildControlGetNumRanks() - how many ranks the guild has.
                //
                // This answered zero, and every other verb in this block
                // worked. The panel builds its rank dropdown with
                // `for i = 1, GuildControlGetNumRanks()`, so the loop never ran
                // once: an empty dropdown, no rank to select, and therefore no
                // rank to edit, promote to or demote to. friendsframe also
                // takes `GuildControlGetNumRanks() - 1` as the lowest rank
                // index, which was minus one.
                //
                // Counted from the roster's ranks rather than the ten name
                // slots in the guild query, because that is where
                // GuildControlGetRankFlags reads rights from - a rank listed
                // here whose flags come back empty is worse than one not
                // listed. The query's names fill in before any roster has
                // arrived, and it pads to ten with empty strings, so those are
                // counted only up to the last one that has a name.
                {"GuildControlGetNumRanks", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) return luaReturnZero(L);
            const auto& ranks = gh->getGuildRoster().ranks;
            if (!ranks.empty()) {
                lua_pushnumber(L, static_cast<double>(ranks.size()));
                return 1;
            }
            const auto& names = gh->getGuildRankNames();
            size_t named = 0;
            for (size_t i = 0; i < names.size(); ++i)
                if (!names[i].empty()) named = i + 1;
            lua_pushnumber(L, static_cast<double>(named));
            return 1;
        }},
                {"GuildControlGetRankName", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int idx = static_cast<int>(luaL_optnumber(L, 1, 0));
            if (!gh || idx < 1) return luaReturnNil(L);
            const auto& ranks = gh->getGuildRankNames();
            if (idx > static_cast<int>(ranks.size())) return luaReturnNil(L);
            lua_pushstring(L, ranks[static_cast<size_t>(idx) - 1].c_str());
            return 1;
        }},
                // GuildControlSetRank(index) - which rank the panel is editing.
                // Remembered rather than sent: GuildControlGetRankFlags below
                // takes no argument, so this is the only thing that says which
                // rank it is being asked about.
                {"GuildControlSetRank", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int idx = static_cast<int>(luaL_optnumber(L, 1, 0));
            if (gh && idx >= 1) gh->setSelectedGuildRank(idx);
            return 0;
        }},
                // GuildControlGetRankFlags() → one boolean per permission
                // checkbox, in the panel's order. See kGuildRankFlagBits for
                // what that order is and why it is not the mask's.
                //
                // All seventeen, because the consumer loops select('#', ...)
                // and finds each checkbox by counting: an answer thirteen long
                // never reaches boxes 15 and 16, the guild bank's repair and
                // withdraw-gold rights, and WithdrawGoldEditBox_Update enables
                // itself off their checked state. Stopping short does not leave
                // those two clear, it leaves them showing the previous rank's.
                {"GuildControlGetRankFlags", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) return 0;
            const auto& roster = gh->getGuildRoster();
            const int sel = gh->getSelectedGuildRank();
            if (sel < 1 || sel > static_cast<int>(roster.ranks.size())) return 0;
            const uint32_t rights = roster.ranks[static_cast<size_t>(sel) - 1].rights;
            for (uint32_t bit : kGuildRankFlagBits) lua_pushboolean(L, (rights & bit) ? 1 : 0);
            return static_cast<int>(kGuildRankFlagCount);
        }},
                // CONFIRM_BUY_GUILDBANK_TAB's accept, from the guild bank's
                // "purchase tab" button. Same story: the packet builder and
                // the handler verb were both here already.
                {"BuyGuildBankTab", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L)) gh->buyGuildBankTab();
            return 0;
        }},
                // CONFIRM_GUILD_DISBAND's accept. The verb has existed the
                // whole time; nothing named it.
                {"GuildDisband", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L)) gh->disbandGuild();
            return 0;
        }},
                {"GuildControlAddRank", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* name = luaL_optstring(L, 1, "");
            if (gh && name && *name) gh->addGuildRank(name);
            return 0;
        }},
                {"GuildControlDelRank", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (gh) gh->delGuildRank();
            return 0;
        }},
                // GuildControlSetRankFlag(checkboxIndex, checked) - stage one
                // permission. Called from the checkbox's own OnClick in the
                // XML, which is why it never appeared in a scan of the Lua.
                //
                // Same order as GuildControlGetRankFlags reads them, off the
                // same table, and for the same reason: the panel's order is not
                // the bit order.
                {"GuildControlSetRankFlag", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int idx = static_cast<int>(luaL_optnumber(L, 1, 0));
            const bool on = lua_toboolean(L, 2) != 0;
            if (!gh || idx < 1 || idx > static_cast<int>(kGuildRankFlagCount)) return 0;
            const uint32_t bit = kGuildRankFlagBits[static_cast<size_t>(idx) - 1];
            // Slot 14 has no bit to set. Nothing calls it - the checkbox that
            // would is not in the frame - but a zero bit here would clear
            // nothing on the way in and revoke nothing on the way out, so the
            // early return keeps that accident from ever being silent.
            if (bit == 0) return 0;
            auto& p = gh->pendingGuildRankRef();
            if (on) p.rights |= bit; else p.rights &= ~bit;
            return 0;
        }},
                // GuildControlSaveRank(name) - commit the staged rank.
                //
                // One packet rewrites the rank whole: rights, name, gold per
                // day and all six bank tabs. A field left at zero is not
                // "unchanged", it is revoked - which is why the staging copy is
                // seeded from the rank as it stands when the panel selects it,
                // and only what the panel edits moves.
                {"GuildControlSaveRank", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* name = luaL_optstring(L, 1, "");
            if (gh && name && *name) gh->saveGuildRank(name);
            return 0;
        }},
                // JoinPermanentChannel(name, password, frameId, permanent)
                //   → zoneChannel, channelName
                //
                // The same join as JoinChannelByName, plus the chat frame the
                // channel should show in. The frame binding is handled entirely
                // in FrameXML, so only the join happens here.
                //
                // The first value says whether this is one of the automatic
                // zone channels; nothing joined by name is, so it is nil rather
                // than a zero that would read as true.
                {"JoinPermanentChannel", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* name = luaL_optstring(L, 1, "");
            const char* pass = luaL_optstring(L, 2, "");
            if (!gh || !name || !*name) return luaReturnNil(L);
            gh->joinChannel(name, pass ? pass : "");
            lua_pushnil(L);
            lua_pushstring(L, name);
            return 2;
        }},
                // GetChannelList() → id, name per channel, and a disabled
                // flag with them from 4.0 on.
                //
                // Three was answered to every interface, and 1.12 through
                // 3.3.5 all read two at a time - so the flag was taken for the
                // next channel's id. 1.12's FCFDropDown_LoadChannels is
                // `for i=1, arg.n, 2` and writes `info.value = "CHANNEL"..arg[i]`;
                // 3.3.5's CreateChatChannelList is that same loop written with
                // select. From the second channel on, both concatenated a
                // boolean and took the handler down with them: the chat menu's
                // channel list on turtle, and the chat config panel's on wotlk.
                //
                // The flag is 4.x's, so 4.x keeps it. Nothing here has been
                // read against a 4.3.4 interface and the answer it already had
                // is the one it should keep until something has.
                {"GetChannelList", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) return 0;
            const auto& joined = gh->getJoinedChannels();
            const int per = interfaceVersion(L) >= 40000 ? 3 : 2;
            // Lua guarantees a C function twenty free slots and no more, so
            // seven channels ran past the end of three-value answers and ten
            // runs past it for two - and the default set plus a couple of
            // custom ones is more than either.
            if (!joined.empty() &&
                !lua_checkstack(L, static_cast<int>(joined.size()) * per)) {
                return 0;
            }
            for (size_t i = 0; i < joined.size(); ++i) {
                lua_pushnumber(L, static_cast<lua_Number>(i + 1));
                lua_pushstring(L, joined[i].c_str());
                if (per == 3) lua_pushboolean(L, 0);   // disabled
            }
            return static_cast<int>(joined.size()) * per;
        }},
                // Channel moderation is not modelled, so the player owns none.
                // IsDisplayChannelOwner lives in lua_system_api.cpp, beside
                // the selected-channel state it has to read. Two registrations
                // of one name would be settled by load order.
                // ChangeChatColor(type, r, g, b) - recolour one kind of chat
                // message.
                //
                // The colour lives in FrameXML's ChatTypeInfo table, which its
                // UPDATE_CHAT_COLOR handler fills from the event's own
                // arguments, so firing the event is the whole of the change.
                //
                // Three decimal places, because an event argument is only read
                // back as a number when it is under twelve characters - a full
                // float would arrive as a string and be assigned straight into
                // a colour field.
                {"ChangeChatColor", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* type = luaL_optstring(L, 1, "");
            if (!gh || !type || !*type) return 0;
            char r[16], g[16], b[16];
            std::snprintf(r, sizeof(r), "%.3f", luaL_optnumber(L, 2, 1.0));
            std::snprintf(g, sizeof(g), "%.3f", luaL_optnumber(L, 3, 1.0));
            std::snprintf(b, sizeof(b), "%.3f", luaL_optnumber(L, 4, 1.0));
            gh->fireAddonEvent("UPDATE_CHAT_COLOR", {type, r, g, b});
            return 0;
        }},
                // UnitIsRaidOfficer(unit) → whether that unit is an assistant.
                // The bit rides along with the group list, same as the main
                // tank and main assist flags read above.
                {"UnitIsRaidOfficer", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* unit = luaL_optstring(L, 1, "");
            if (!gh || !unit || !*unit) return luaReturnFalse(L);
            std::string uid(unit);
            toLowerInPlace(uid);
            const uint64_t guid = resolveUnitGuid(gh, uid);
            if (guid == 0) return luaReturnFalse(L);
            for (const auto& mem : gh->getPartyData().members) {
                if (mem.guid == guid) { lua_pushboolean(L, (mem.flags & 0x01) ? 1 : 0); return 1; }
            }
            return luaReturnFalse(L);
        }},
                // ---- Loot rules and party assignments ----
                //
                // SetLootMethod(method, masterPlayer) and SetLootThreshold(q)
                // both write CMSG_LOOT_METHOD, which carries all three settings
                // at once - so each resends the other two as they stand rather
                // than clearing them.
                {"SetLootMethod", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) return 0;
            std::string method(luaL_optstring(L, 1, ""));
            toLowerInPlace(method);
            const uint8_t m = lootMethodFromToken(method);

            const auto& pd = gh->getPartyData();
            uint64_t masterGuid = 0;
            if (m == 2) {
                // The second argument is a unit id or a name, depending on who
                // is calling - the loot dropdown passes a name.
                const char* who = luaL_optstring(L, 2, nullptr);
                if (who && *who) {
                    for (const auto& mem : pd.members) {
                        if (mem.name == who) { masterGuid = mem.guid; break; }
                    }
                    if (masterGuid == 0) masterGuid = gh->getPlayerGuid();
                }
            }
            gh->setLootMethod(m, masterGuid, pd.lootThreshold);
            return 0;
        }},
                {"SetLootThreshold", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) return 0;
            const auto quality = static_cast<uint8_t>(luaL_optnumber(L, 1, 2));
            const auto& pd = gh->getPartyData();
            gh->setLootMethod(pd.lootMethod, pd.looterGuid, quality);
            return 0;
        }},
                // GetPartyAssignment(assignment, unit) → whether that unit
                // holds it. The flags come with the group list.
                {"GetPartyAssignment", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            std::string what(luaL_optstring(L, 1, ""));
            const char* unit = luaL_optstring(L, 2, "");
            toLowerInPlace(what);
            if (!gh || !unit || !*unit) return luaReturnFalse(L);
            const uint8_t bit = (what == "maintank") ? 0x02
                              : (what == "mainassist") ? 0x04 : 0x00;
            if (bit == 0) return luaReturnFalse(L);
            std::string uid(unit);
            toLowerInPlace(uid);
            const uint64_t guid = resolveUnitGuid(gh, uid);
            if (guid == 0) return luaReturnFalse(L);
            for (const auto& mem : gh->getPartyData().members) {
                if (mem.guid == guid) { lua_pushboolean(L, (mem.flags & bit) ? 1 : 0); return 1; }
            }
            return luaReturnFalse(L);
        }},
                {"SetPartyAssignment", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            std::string what(luaL_optstring(L, 1, ""));
            const char* unit = luaL_optstring(L, 2, "");
            toLowerInPlace(what);
            if (!gh || !unit || !*unit) return 0;
            const int assignment = (what == "maintank") ? 0 : (what == "mainassist") ? 1 : -1;
            if (assignment < 0) return 0;
            std::string uid(unit);
            toLowerInPlace(uid);
            const uint64_t guid = resolveUnitGuid(gh, uid);
            if (guid) gh->setPartyAssignment(static_cast<uint8_t>(assignment), guid, true);
            return 0;
        }},
                {"ClearPartyAssignment", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            std::string what(luaL_optstring(L, 1, ""));
            const char* unit = luaL_optstring(L, 2, "");
            toLowerInPlace(what);
            if (!gh || !unit || !*unit) return 0;
            const int assignment = (what == "maintank") ? 0 : (what == "mainassist") ? 1 : -1;
            if (assignment < 0) return 0;
            std::string uid(unit);
            toLowerInPlace(uid);
            const uint64_t guid = resolveUnitGuid(gh, uid);
            if (guid) gh->setPartyAssignment(static_cast<uint8_t>(assignment), guid, false);
            return 0;
        }},
                // ---- Raid group management ----
                // SetRaidSubgroup(raidIndex, group) - move a member into a
                // different group of eight. SwapRaidSubgroup exchanges two,
                // which is what the raid UI uses when the destination is full.
                // The group number stays as the interface counts it, from one.
                // SocialHandler::setRaidSubgroup takes 1..8 and writes group - 1
                // itself - subtracting here as well makes Group 1 a zero, which
                // that function then rejects, and every other group land one
                // short. The wire is zero-based; the conversion is just already
                // done a layer down.
                {"SetRaidSubgroup", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int idx   = static_cast<int>(luaL_optnumber(L, 1, 0));
            const int group = static_cast<int>(luaL_optnumber(L, 2, 0));
            if (!gh || idx < 1) return 0;
            const auto& members = gh->getPartyData().members;
            if (idx > static_cast<int>(members.size())) return 0;
            gh->setRaidSubgroup(members[static_cast<size_t>(idx) - 1].name,
                                static_cast<uint8_t>(group));
            return 0;
        }},
                {"SwapRaidSubgroup", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int a = static_cast<int>(luaL_optnumber(L, 1, 0));
            const int b = static_cast<int>(luaL_optnumber(L, 2, 0));
            if (!gh || a < 1 || b < 1) return 0;
            const auto& members = gh->getPartyData().members;
            const int n = static_cast<int>(members.size());
            if (a > n || b > n) return 0;
            gh->swapRaidSubgroup(members[static_cast<size_t>(a) - 1].name,
                                 members[static_cast<size_t>(b) - 1].name);
            return 0;
        }},
                // Which row the raid roster has highlighted. Purely local -
                // the raid UI reads it back through its own frames, and
                // nothing is sent for it.
                {"SetRaidRosterSelection", [](lua_State* L) -> int { (void)L; return 0; }},
                // Voice chat again: there is none, so nobody is muted or
                // silenced and asking to mute them does nothing.
                {"IsMuted",             luaReturnFalse},
                {"UnitIsSilenced",      luaReturnFalse},
                {"AddOrDelMute",        [](lua_State* L) -> int { (void)L; return 0; }},
                {"ChannelSilenceVoice", [](lua_State* L) -> int { (void)L; return 0; }},
                {"ChannelUnSilenceVoice", [](lua_State* L) -> int { (void)L; return 0; }},
                // Vehicles are not modelled in the raid frames.
                {"UnitTargetsVehicleInRaidUI", luaReturnFalse},
                {"GetNumLanguages",   lua_GetNumLanguages},
                // The 1.12 client's own spelling, and not a typo on this side:
                // stock ChatFrame.lua opens LanguageMenu_LoadLanguages with
                // `local numLanguages = GetNumLaguages();`, so the client it
                // was written against must answer to that name. Missing, the
                // no-op fallback answered something that is not a number and
                // the next line - `for i = 1, numLanguages` - raised, leaving
                // the language menu empty. Reported in #132.
                {"GetNumLaguages",    lua_GetNumLanguages},
                {"GetLanguageByIndex", lua_GetLanguageByIndex},
                {"SendChatMessage",   lua_SendChatMessage},
                {"SendAddonMessage",  lua_SendAddonMessage},
                {"RegisterAddonMessagePrefix", lua_RegisterAddonMessagePrefix},
                {"IsAddonMessagePrefixRegistered", lua_IsAddonMessagePrefixRegistered},
                {"IsInGuild",               lua_IsInGuild},
                {"GetGuildInfo",            lua_GetGuildInfoFunc},
                {"GetNumGuildMembers",      lua_GetNumGuildMembers},
                {"GuildRoster", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (gh) gh->requestGuildRoster();
            return 0;
        }},
                // SortGuildRoster(field) - one of name, zone, level, class,
                // rank, note, online, from whichever column header was
                // clicked. Clicking the same one again reverses it, which is
                // the header's only way of saying so: it sends the field and
                // nothing else, and expects the API to remember.
                {"SortGuildRoster", [](lua_State* L) -> int {
            const std::string field = luaL_optstring(L, 1, "name");
            if (field == guildSortField()) guildSortReverse() = !guildSortReverse();
            else { guildSortField() = field; guildSortReverse() = false; }
            return 0;
        }},
                // The three answers to a battlefield manager prompt. All were
                // unbound, so FrameXML's BFMGR dialogs raised on accept - and
                // the client fires the events that put them on screen.
                {"BattlefieldMgrEntryInviteResponse", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) return 0;
            if (lua_toboolean(L, 2)) gh->acceptBfMgrInvite();
            else                     gh->declineBfMgrInvite();
            return 0;
        }},
                {"BattlefieldMgrQueueInviteResponse", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) return 0;
            const auto battleId = static_cast<uint32_t>(luaL_optnumber(L, 1, 0));
            gh->respondBfMgrQueueInvite(battleId ? battleId : gh->getBfMgrBattleId(),
                                        lua_toboolean(L, 2) != 0);
            return 0;
        }},
                {"BattlefieldMgrExitRequest", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) return 0;
            const auto battleId = static_cast<uint32_t>(luaL_optnumber(L, 1, 0));
            gh->requestBfMgrExit(battleId ? battleId : gh->getBfMgrBattleId());
            return 0;
        }},
                // The area spirit healer, which is how anyone gets up again
                // in a battleground. None of the three existed, and
                // UIParent_OnEvent calls the first of them the moment the
                // healer is in range.
                {"AcceptAreaSpiritHeal", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L)) gh->queueAreaSpiritHeal();
            return 0;
        }},
                // There is no opcode for leaving the queue - AzerothCore drops
                // a player from it when they leave the battleground and at no
                // other time - so this accepts the click and does nothing,
                // which is the honest answer rather than a raise.
                {"CancelAreaSpiritHeal", [](lua_State* L) -> int { (void)L; return 0; }},
                {"GetAreaSpiritHealerTime", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushnumber(L, gh ? gh->getAreaSpiritHealerTime() : 0.0f);
            return 1;
        }},
                {"GetGuildRosterInfo",      lua_GetGuildRosterInfo},
                {"SetGuildRosterSelection", [](lua_State* L) -> int {
            selectedGuildRosterRow() = static_cast<int>(luaL_optnumber(L, 1, 0));
            return 0;
        }},
                {"GetGuildRosterSelection", [](lua_State* L) -> int {
            lua_pushnumber(L, selectedGuildRosterRow());
            return 1;
        }},
                // The two note editors. Both dialogs read the selection to
                // know whose note they are editing and then send by index;
                // this client's own verbs take a name, so the row is resolved
                // here. Neither was bound at all, so both dialogs accepted
                // text and threw it away.
                {"GuildRosterSetPublicNote", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const std::string name = guildRosterNameAt(gh, static_cast<int>(luaL_optnumber(L, 1, 0)));
            if (!name.empty()) gh->setGuildPublicNote(name, luaL_optstring(L, 2, ""));
            return 0;
        }},
                {"GuildRosterSetOfficerNote", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const std::string name = guildRosterNameAt(gh, static_cast<int>(luaL_optnumber(L, 1, 0)));
            if (!name.empty()) gh->setGuildOfficerNote(name, luaL_optstring(L, 2, ""));
            return 0;
        }},
                {"GetGuildRosterMOTD",      lua_GetGuildRosterMOTD},
                {"GetNumFriends",           lua_GetNumFriends},
                {"GetFriendInfo",           lua_GetFriendInfo},
                // SetFriendNotes(index or name, note) - the note dialog's
                // accept. Unbound, so typing a note and pressing Okay did
                // nothing and the old note came back on the next roster
                // update.
                {"SetFriendNotes", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* note = lua_tostring(L, 2);
            if (!gh || !note) return 0;
            std::string name;
            if (lua_isstring(L, 1) && !lua_isnumber(L, 1)) {
                name = lua_tostring(L, 1);
            } else {
                const int idx = static_cast<int>(luaL_optnumber(L, 1, 0));
                int found = 0;
                for (const auto& c : gh->getContacts()) {
                    if (!c.isFriend()) continue;
                    if (++found == idx) { name = c.name; break; }
                }
            }
            if (!name.empty()) gh->setFriendNote(name, note);
            return 0;
        }},
                {"GetNumIgnores",           lua_GetNumIgnores},
                {"GetIgnoreName",           lua_GetIgnoreName},
                {"GuildInvite", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (gh) gh->inviteToGuild(luaL_checkstring(L, 1));
            return 0;
        }},
                {"GuildUninvite", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (gh) gh->kickGuildMember(luaL_checkstring(L, 1));
            return 0;
        }},
                {"GuildPromote", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (gh) gh->promoteGuildMember(luaL_checkstring(L, 1));
            return 0;
        }},
                {"GuildDemote", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (gh) gh->demoteGuildMember(luaL_checkstring(L, 1));
            return 0;
        }},
                {"GuildLeave", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (gh) gh->leaveGuild();
            return 0;
        }},
                {"GuildSetPublicNote", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (gh) gh->setGuildPublicNote(luaL_checkstring(L, 1), luaL_checkstring(L, 2));
            return 0;
        }},
                // DoEmote(token) - every emote FrameXML offers, and there
                // are three hundred and twenty-three of them.
                //
                // This was a hand-written map of thirty-one names, and a token
                // outside it fell off the end of the lookup and returned
                // having done nothing at all: no packet, no animation, no
                // text. /rasp, /moo, /train and roughly two hundred and ninety
                // others simply did not happen, and nothing said so.
                //
                // It went unnoticed because this client's own chat never
                // called DoEmote - it has an emote fallthrough of its own that
                // reads the same registry directly. Handing chat over made
                // DoEmote the only route, and shrank the emote set to
                // thirty-one on the way through.
                //
                // EmoteRegistry keys its table on lower-cased command names
                // out of EmotesText.dbc; FrameXML's tokens are upper case.
                //
                // Only the id is needed: the server echoes SMSG_EMOTE back and
                // chat_handler plays the animation from it, and SMSG_TEXT_EMOTE
                // carries the wording. Sending is the whole of the client's
                // part.
                {"DoEmote", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* token = luaL_checkstring(L, 1);
            if (!gh || !token) return 0;
            std::string name(token);
            toLowerInPlace(name);
            const uint32_t emoteId = rendering::AnimationController::getEmoteDbcId(name);
            if (emoteId == 0) return 0;
            // The second argument is whoever the emote is aimed at, and it was
            // read by nothing - so "/wave Bob" waved at the current target, or
            // at nobody, whatever Bob was standing there. chatframe hands over
            // the text after the command, which may be a unit token or a name.
            //
            // The current target when none is named, which is what the emote
            // buttons and a bare "/dance" mean.
            const char* aimedAt = luaL_optstring(L, 2, "");
            uint64_t target = (aimedAt && *aimedAt) ? resolveUnitOrName(gh, aimedAt) : 0;
            if (target == 0 && (!aimedAt || !*aimedAt)) {
                target = gh->hasTarget() ? gh->getTargetGuid() : 0;
            }
            gh->sendTextEmote(emoteId, target);
            // And the animation, which this did not do. Telling the server is
            // the half that is seen by everyone else; playing it is the half
            // seen by the player, and standing still while the room is told you
            // danced is what an emote that "does nothing" looks like.
            if (auto* svc = getLuaServices(L); svc && svc->playEmoteAnimation) {
                svc->playEmoteAnimation(name);
            }
            return 0;
        }},
                {"AddFriend", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* name = luaL_checkstring(L, 1);
            const char* note = luaL_optstring(L, 2, "");
            if (gh) gh->addFriend(name, note);
            return 0;
        }},
                {"RemoveFriend", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* name = luaL_checkstring(L, 1);
            if (gh) gh->removeFriend(name);
            return 0;
        }},
                {"AddIgnore", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* name = luaL_checkstring(L, 1);
            if (gh) gh->addIgnore(name);
            return 0;
        }},
                {"DelIgnore", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* name = luaL_checkstring(L, 1);
            if (gh) gh->removeIgnore(name);
            return 0;
        }},
                // ShowFriends() - not a panel verb despite the name: it asks
                // the server to resend the friend, ignore and mute lists, and
                // FriendsFrame_Update calls it every time the list redraws.
                // It did nothing, on the reading that the panel is drawn by
                // this client rather than by Lua - but what it is for is the
                // request, and without it the list was whatever had last been
                // pushed, with an online column that went stale and stayed
                // stale until a friend happened to log in or out.
                //
                // CMSG_CONTACT_LIST was in all three opcode maps with nothing
                // building it.
                {"ShowFriends", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L)) gh->requestContactList();
            return 0;
        }},
                {"GetNumWhoResults", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) { lua_pushnumber(L, 0); lua_pushnumber(L, 0); return 2; }
            lua_pushnumber(L, gh->getWhoResults().size());
            lua_pushnumber(L, gh->getWhoOnlineCount());
            return 2;
        }},
                {"GetWhoInfo", [](lua_State* L) -> int {
            // GetWhoInfo(index) → name, guild, level, race, class, zone, classFileName
            auto* gh = getGameHandler(L);
            int index = static_cast<int>(luaL_checknumber(L, 1));
            if (!gh || index < 1) { return luaReturnNil(L); }
            const auto view = whoResultsView(gh);
            if (index > static_cast<int>(view.size())) { return luaReturnNil(L); }
            const auto& w = *view[static_cast<size_t>(index) - 1];


            const char* raceName = (w.raceId < 12) ? kLuaRaces[w.raceId] : "Unknown";
            const char* className = (w.classId < 12) ? kLuaClasses[w.classId] : "Unknown";
            lua_pushstring(L, w.name.c_str());
            lua_pushstring(L, w.guildName.c_str());
            lua_pushnumber(L, w.level);
            lua_pushstring(L, raceName);
            lua_pushstring(L, className);
            lua_pushstring(L, ""); // zone name (would need area lookup)
            // The who list reads this exactly as the guild roster does, so it
            // gets the same answer from the same place. It used to keep a
            // private copy of the token table with a fallback of "WARRIOR",
            // which paints an unknown class a confident wrong colour.
            luaPushClassToken(L, w.classId);
            return 7;
        }},
                {"SendWho", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* query = luaL_optstring(L, 1, "");
            if (gh) gh->queryWho(query);
            return 0;
        }},
                // SetWhoToUI(toUI) - whether a /who answer should fill the
                // panel or be printed as chat lines. Nothing to switch: this
                // client's who handler only ever stores the rows and fires
                // WHO_LIST_UPDATE, so the panel is already the one place they
                // go and there is no chat print to turn off.
                // SetWhoToUI(toUI) - where a /who answer should go.
                //
                // FriendsFrame turns this on while its Who tab is up and off
                // again on the way out. A no-op meant the client never printed
                // results to chat, so a /who typed with the panel closed
                // answered nothing at all: the rows were parsed and stored and
                // kept, which looks exactly like a search the server ignored.
                {"SetWhoToUI", [](lua_State* L) -> int {
            // A number, not a boolean: FrameXML passes 1 and 0, and zero is
            // true in Lua, so lua_toboolean would read "off" as "on" and this
            // would be a no-op with extra steps.
            if (auto* gh = getGameHandler(L)) {
                gh->setWhoToUI(luaOptNumberText(L, 1, 0.0) != 0.0);
            }
            return 0;
        }},
                {"GetNumGossipOptions", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushnumber(L, gh ? gh->getCurrentGossip().options.size() : 0);
            return 1;
        }},
                // The gossip window's two quest lists. Both are flat runs of
                // values rather than tables - five per available quest and four
                // per active one, which is the stride GossipFrame reads them at.
                {"GetGossipAvailableQuests", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) return 0;
            // Five values per quest, and the quest count is the server's. Room
            // first: pushing past the stack top corrupts the heap rather than
            // raising.
            if (!lua_checkstack(
                    L, static_cast<int>(gh->getCurrentGossip().quests.size()) * 5 + 1)) {
                return 0;
            }
            int n = 0;
            for (const auto& q : gh->getCurrentGossip().quests) {
                if (!gossipQuestIsAvailable(q.questIcon)) continue;
                lua_pushstring(L, q.title.c_str());
                lua_pushnumber(L, q.questLevel);
                // Trivial greys the title out. Nothing here knows a quest's
                // green range, and guessing it would grey quests that are not.
                lua_pushboolean(L, 0);
                lua_pushboolean(L, (q.questFlags & kQuestFlagsDaily) ? 1 : 0);
                lua_pushboolean(L, q.isRepeatable ? 1 : 0);
                n += 5;
            }
            return n;
        }},
                {"GetGossipActiveQuests", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) return 0;
            // Five values per quest, and the quest count is the server's. Room
            // first: pushing past the stack top corrupts the heap rather than
            // raising.
            if (!lua_checkstack(
                    L, static_cast<int>(gh->getCurrentGossip().quests.size()) * 5 + 1)) {
                return 0;
            }
            int n = 0;
            for (const auto& q : gh->getCurrentGossip().quests) {
                if (!gossipQuestIsActive(q.questIcon)) continue;
                lua_pushstring(L, q.title.c_str());
                lua_pushnumber(L, q.questLevel);
                lua_pushboolean(L, 0);
                lua_pushboolean(L, gossipQuestIsCompletable(q.questIcon) ? 1 : 0);
                n += 4;
            }
            return n;
        }},
                {"SelectGossipAvailableQuest", [](lua_State* L) -> int {
            return selectGossipQuestAt(L, /*available=*/true);
        }},
                {"SelectGossipActiveQuest", [](lua_State* L) -> int {
            return selectGossipQuestAt(L, /*available=*/false);
        }},
                // The greeting panel, shown when a quest giver has several
                // quests and nothing else to say. It reads the same list as the
                // gossip window - this client routes both through one - but
                // asks for it a quest at a time rather than all at once.
                {"GetGreetingText", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushstring(L, gh ? gh->getQuestGreeting().c_str() : "");
            return 1;
        }},
                {"GetNumAvailableQuests", [](lua_State* L) -> int {
            lua_pushnumber(L, countGossipQuests(L, /*available=*/true));
            return 1;
        }},
                {"GetNumActiveQuests", [](lua_State* L) -> int {
            lua_pushnumber(L, countGossipQuests(L, /*available=*/false));
            return 1;
        }},
                {"GetAvailableTitle", [](lua_State* L) -> int {
            return pushGossipQuestTitle(L, /*available=*/true);
        }},
                {"GetActiveTitle", [](lua_State* L) -> int {
            return pushGossipQuestTitle(L, /*available=*/false);
        }},
                {"SelectAvailableQuest", [](lua_State* L) -> int {
            return selectGossipQuestAt(L, /*available=*/true);
        }},
                {"SelectActiveQuest", [](lua_State* L) -> int {
            return selectGossipQuestAt(L, /*available=*/false);
        }},
                // Greys the title of a quest the player has outlevelled. The
                // gossip list carries each quest's level and the threshold is
                // the server's own formula, so this is a comparison rather than
                // the guess the old comment here declined to make.
                {"IsActiveQuestTrivial", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int index = static_cast<int>(luaL_checknumber(L, 1));
            const auto* q = gossipQuestAt(L, /*available=*/false, index);
            lua_pushboolean(L, (gh && q &&
                questIsTrivial(static_cast<int>(gh->getPlayerLevel()), q->questLevel)) ? 1 : 0);
            return 1;
        }},
                // isTrivial, isDaily, isRepeatable - which icon the greeting
                // panel puts beside an offered quest.
                {"GetAvailableQuestInfo", [](lua_State* L) -> int {
            const int index = static_cast<int>(luaL_checknumber(L, 1));
            const auto* q = gossipQuestAt(L, /*available=*/true, index);
            auto* gh = getGameHandler(L);
            lua_pushboolean(L, (gh && q &&
                questIsTrivial(static_cast<int>(gh->getPlayerLevel()), q->questLevel)) ? 1 : 0);
            lua_pushboolean(L, q && (q->questFlags & kQuestFlagsDaily) ? 1 : 0);
            lua_pushboolean(L, q && q->isRepeatable ? 1 : 0);
            return 3;
        }},
                // Whether the gossip window must be shown even when there is
                // one thing to click. The server sends no such flag here, and
                // the frame reads it as "not ForceGossip()" to decide whether
                // to go straight to a lone vendor or flight master - which is
                // what the real client does, so a definite no keeps that.
                // GuildSetMOTD(text) - what /gmotd does
                //
                // The rest of the guild commands were already bound; this one
                // was not, and the server refuses it from a rank without the
                // right, which is where that decision belongs while nothing
                // here reads rank rights.
                {"GuildSetMOTD", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* motd = luaL_optstring(L, 1, "");
            if (gh && motd) gh->setGuildMotd(motd);
            return 0;
        }},
                {"SendSystemMessage",   lua_SendSystemMessage},
                {"GetSelectedFriend", [](lua_State* L) -> int {
            lua_pushnumber(L, selectedFriend()); return 1; }},
                {"SetSelectedFriend", [](lua_State* L) -> int {
            selectedFriend() = static_cast<int>(luaL_optnumber(L, 1, 0)); return 0; }},
                {"GetSelectedIgnore", [](lua_State* L) -> int {
            lua_pushnumber(L, selectedIgnore()); return 1; }},
                {"SetSelectedIgnore", [](lua_State* L) -> int {
            selectedIgnore() = static_cast<int>(luaL_optnumber(L, 1, 0)); return 0; }},
                {"AddOrRemoveFriend",   lua_AddOrRemoveFriend},
                {"AddOrDelIgnore",      lua_AddOrDelIgnore},
                // DoReadyCheck() - what /readycheck does
                {"DoReadyCheck", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L)) gh->initiateReadyCheck();
            return 0;
        }},
                // ConfirmReadyCheck(ready) - the Yes and No buttons on
                // FrameXML's ReadyCheckFrame. readycheck.xml passes 1 for yes
                // and passes NOTHING for no, so the absent argument has to read
                // as false rather than as a missing parameter.
                {"ConfirmReadyCheck", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L)) {
                gh->respondToReadyCheck(lua_toboolean(L, 1) != 0);
                gh->dismissReadyCheck();
            }
            return 0;
        }},
                // The buttons on FrameXML's summon and talent-wipe popups.
                // Both prompts are raised - uiparent.lua answers CONFIRM_SUMMON
                // and CONFIRM_TALENT_WIPE, and this client fires both - so
                // without these the popup appeared and neither button did
                // anything. The client's own dialogs were the only way to
                // answer either, which is the shape this branch keeps finding:
                // a capability reachable from one of this client's own windows
                // and from nowhere else.
                {"ConfirmSummon", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L)) gh->acceptSummon();
            return 0;
        }},
                {"CancelSummon", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L)) gh->declineSummon();
            return 0;
        }},
                // The popup's own OnCancel only hides the talent frame, so
                // there is no DeclineTalentWipe to bind - cancelTalentWipe is
                // this client's bookkeeping and runs when the popup goes.
                {"ConfirmTalentWipe", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L)) gh->confirmTalentWipe();
            return 0;
        }},
                {"AcceptGroup",         lua_AcceptGroup},
                {"DeclineGroup",        lua_DeclineGroup},
                {"AcceptGuild",         lua_AcceptGuild},
                {"DeclineGuild",        lua_DeclineGuild},
                {"AcceptResurrect",     lua_AcceptResurrect},
                // --- The spirit healer ---
                //
                // Its prompt is the XP_LOSS popup, raised from CONFIRM_XP_LOSS.
                // activateSpiritHealer had exactly one caller, this client's
                // own gossip window, so with gossip handed over nothing could
                // reach it and standing at a spirit healer did nothing.
                {"AcceptXPLoss", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L)) {
                const uint64_t healer = gh->resurrectCasterGuidRef();
                if (healer) gh->activateSpiritHealer(healer);
                gh->resurrectRequestPendingRef() = false;
            }
            return 0;
        }},
                // The popup hides itself and closes the gossip when this turns
                // false, so it answers true for as long as the offer stands.
                // The server withdraws the offer by other means; there is no
                // distance to measure against a healer this client is not
                // tracking the position of.
                {"CheckSpiritHealerDist", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushboolean(L, gh && gh->resurrectRequestPendingRef() &&
                                gh->resurrectIsSpiritHealerRef() ? 1 : 0);
            return 1;
        }},
                // GetResSicknessDuration() → how long the sickness lasts, as
                // the text this goes into is "%s of Resurrection Sickness".
                //
                // The server's own rule (Player::ResurrectPlayer): below the
                // sickness level there is none, 11 to 19 is a minute for each
                // level above ten, and twenty and up is the full ten minutes.
                // nil is what "none" looks like here, and it picks the
                // no-sickness wording rather than printing "0 min".
                {"GetResSicknessDuration", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const uint32_t level = gh ? gh->getPlayerLevel() : 0;
            constexpr uint32_t kSicknessLevel = 11;
            if (level < kSicknessLevel) { lua_pushnil(L); return 1; }
            const uint32_t minutes = level >= kSicknessLevel + 9
                ? 10u : (level - kSicknessLevel + 1u);
            lua_pushstring(L, (std::to_string(minutes) + " min").c_str());
            return 1;
        }},
                {"DeclineResurrect",    lua_DeclineResurrect},
                {"AcceptDuel",          lua_AcceptDuel},
                {"CancelDuel",          lua_CancelDuel},
                {"GetAutoCompleteResults", lua_GetAutoCompleteResults},
                // Every "faction index" below is a position in the *drawn*
                // rows, not in the flat list the server sent: headers are rows
                // too, and a collapsed one takes its children out of the list
                // entirely. Resolving one against the flat list would act on
                // whatever faction happened to sit at that offset instead.
                {"GetNumFactions", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushnumber(L, gh ? static_cast<double>(gh->getReputationRows().size()) : 0.0);
            return 1;
        }},
                // name, description, standingID, barMin, barMax, barValue,
                // atWarWith, canToggleAtWar, isHeader, isCollapsed, hasRep,
                // isWatched, isChild
                {"GetFactionInfo", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int index = static_cast<int>(luaL_checknumber(L, 1));
            if (!gh || index < 1) { return luaReturnNil(L); }
            const auto& rows = gh->getReputationRows();
            if (index > static_cast<int>(rows.size())) { return luaReturnNil(L); }
            return pushFactionRow(L, gh, rows[index - 1]);
        }},
                // The reputation tab's controls. Every one of these has a verb
                // on GameHandler and none had a binding, so the tab could show
                // a standing and change nothing about it - this client's own
                // reputation panel was the only way to declare war on a faction
                // or pick which bar to watch.
                //
                // All take a position in the drawn rows above. A header row
                // the player has no standing with carries reputationIndex 0
                // and cannot be acted on, which is why each checks hasRep
                // before sending anything: declaring war on "Classic" is not a
                // request the server has an answer for.
                {"SetWatchedFactionIndex", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int index = static_cast<int>(luaL_optnumber(L, 1, 0));
            if (!gh) return 0;
            // Zero is FrameXML's "watch nothing", not a missing argument.
            if (index < 1) { gh->setWatchedFactionId(0); return 0; }
            const auto& rows = gh->getReputationRows();
            if (index <= static_cast<int>(rows.size())) {
                const auto& row = rows[static_cast<size_t>(index) - 1];
                if (row.hasRep) gh->setWatchedFactionId(row.factionId);
            }
            return 0;
        }},
                {"FactionToggleAtWar", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int index = static_cast<int>(luaL_optnumber(L, 1, 0));
            if (!gh || index < 1) return 0;
            const auto& rows = gh->getReputationRows();
            if (index > static_cast<int>(rows.size())) return 0;
            const auto& f = rows[static_cast<size_t>(index) - 1];
            if (!f.hasRep) return 0;
            const bool atWar = (f.flags & game::GameHandler::FACTION_FLAG_AT_WAR) != 0;
            gh->setFactionAtWar(f.reputationIndex, !atWar);
            return 0;
        }},
                {"SetFactionActive", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int index = static_cast<int>(luaL_optnumber(L, 1, 0));
            if (!gh || index < 1) return 0;
            const auto& rows = gh->getReputationRows();
            if (index <= static_cast<int>(rows.size())) {
                const auto& row = rows[static_cast<size_t>(index) - 1];
                if (row.hasRep) gh->setFactionInactive(row.reputationIndex, false);
            }
            return 0;
        }},
                {"SetFactionInactive", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int index = static_cast<int>(luaL_optnumber(L, 1, 0));
            if (!gh || index < 1) return 0;
            const auto& rows = gh->getReputationRows();
            if (index <= static_cast<int>(rows.size())) {
                const auto& row = rows[static_cast<size_t>(index) - 1];
                if (row.hasRep) gh->setFactionInactive(row.reputationIndex, true);
            }
            return 0;
        }},
                // GetFactionInfo reports every row as isHeader false, because
                // the list it walks is flat - the client does not read each
                // faction's parent to build categories. With no headers there
                // is nothing to open or close, and the tab calls these from a
                // row's click handler regardless of what the row is.
                // Opening and closing a group. FrameXML calls these from a
                // row's click handler regardless of what the row is, so a
                // non-header index is ignored rather than refused.
                {"ExpandFactionHeader", [](lua_State* L) -> int {
            return factionHeaderSetCollapsed(L, false);
        }},
                {"CollapseFactionHeader", [](lua_State* L) -> int {
            return factionHeaderSetCollapsed(L, true);
        }},
                {"IsFactionInactive", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int index = static_cast<int>(luaL_checknumber(L, 1));
            const auto* rows = gh ? &gh->getReputationRows() : nullptr;
            const bool inactive = rows && index >= 1 &&
                index <= static_cast<int>(rows->size()) &&
                ((*rows)[index - 1].flags & game::GameHandler::FACTION_FLAG_INACTIVE) != 0;
            lua_pushboolean(L, inactive ? 1 : 0);
            return 1;
        }},
                {"GetSelectedFaction", [](lua_State* L) -> int {
            lua_pushnumber(L, selectedFaction());
            return 1;
        }},
                {"SetSelectedFaction", [](lua_State* L) -> int {
            selectedFaction() = static_cast<int>(luaL_checknumber(L, 1));
            return 0;
        }},
                {"ForceGossip", [](lua_State* L) -> int {
            lua_pushboolean(L, 0);
            return 1;
        }},
                {"GetGossipOptions", [](lua_State* L) -> int {
            // Returns pairs of (text, type) for each option
            auto* gh = getGameHandler(L);
            if (!gh) return 0;
            const auto& opts = gh->getCurrentGossip().options;
            // Two values per option, and the option count is the server's.
            // Room first: pushing past the stack top corrupts the heap rather
            // than raising.
            if (!lua_checkstack(L, static_cast<int>(opts.size()) * 2 + 1)) return 0;
            int n = 0;
            static constexpr const char* kIcons[] = {"gossip","vendor","taxi","trainer","spiritguide","innkeeper","banker","petition","tabard","battlemaster","auctioneer"};
            for (const auto& o : opts) {
                lua_pushstring(L, o.text.c_str());
                lua_pushstring(L, o.icon < 11 ? kIcons[o.icon] : "gossip");
                n += 2;
            }
            return n;
        }},
                // SelectGossipOption(index, text, confirmed)
                //
                // An option can come with a confirmation box: SMSG_GOSSIP_MESSAGE
                // carries boxText and boxMoney per option, and this client has
                // parsed both for as long as it has read the packet. Nothing
                // raised the prompt, so an option costing a thousand gold - the
                // dual talent specialisation, a guild tabard - was bought by
                // the click that selected it, with no confirmation at all.
                //
                // The third argument is FrameXML saying the player has now
                // answered: GOSSIP_CONFIRM's Accept calls back with it true.
                {"SelectGossipOption", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            int index = static_cast<int>(luaL_checknumber(L, 1));
            if (!gh || index < 1) return 0;
            const auto& opts = gh->getCurrentGossip().options;
            if (index > static_cast<int>(opts.size())) return 0;
            const auto& opt = opts[static_cast<size_t>(index - 1)];
            const bool confirmed = lua_toboolean(L, 3) != 0;

            // A coded option wants a string typed rather than a price agreed,
            // and the interface asks for it with a different dialog.
            if (!confirmed && opt.isCoded) {
                gh->fireAddonEvent("GOSSIP_ENTER_CODE", {std::to_string(index)});
                return 0;
            }
            if (!confirmed && (opt.boxMoney > 0 || !opt.boxText.empty())) {
                // arg1 is handed straight back to this function as the index,
                // so it is the position rather than the option's id.
                gh->fireAddonEvent("GOSSIP_CONFIRM",
                                   {std::to_string(index), opt.boxText,
                                    std::to_string(opt.boxMoney)});
                return 0;
            }
            gh->selectGossipOption(opt.id, luaL_optstring(L, 2, ""));
            return 0;
        }},
                {"GetNumGossipAvailableQuests", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) { return luaReturnZero(L); }
            int count = 0;
            for (const auto& q : gh->getCurrentGossip().quests)
                if (q.questIcon != 4) ++count; // 4 = active/in-progress
            lua_pushnumber(L, count);
            return 1;
        }},
                {"GetNumGossipActiveQuests", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) { return luaReturnZero(L); }
            int count = 0;
            for (const auto& q : gh->getCurrentGossip().quests)
                if (q.questIcon == 4) ++count;
            lua_pushnumber(L, count);
            return 1;
        }},
                {"CloseGossip", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (gh) gh->closeGossip();
            return 0;
        }},
                {"InviteUnit", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (gh) gh->inviteToGroup(luaL_checkstring(L, 1));
            return 0;
        }},
                {"UninviteUnit", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (gh) gh->uninvitePlayer(luaL_checkstring(L, 1));
            return 0;
        }},
                {"LeaveParty", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (gh) gh->leaveGroup();
            return 0;
        }},
                // FollowUnit(unit) - and this one matters more than it looks.
                //
                // FrameXML owns every follow command: SLASH_FOLLOW1 through 7
                // are /f, /follow and /fol, and the client's chat tries
                // SlashCmdList before its own registry and returns as soon as a
                // handler is found - so all three landed on this no-op and the
                // client's working /follow never ran. Following was simply
                // broken from chat.
                //
                // followTarget aims at whatever is targeted, so the cases that
                // mean that are answered and the rest still are not: a named
                // party member who is not the target needs a guid the movement
                // handler has no way to take yet.
                {"FollowUnit", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) return 0;
            std::string who(luaL_optstring(L, 1, ""));
            toLowerInPlace(who);
            const bool meansTarget =
                who.empty() || who == "target" ||
                // A typed name that happens to be the target, which is what
                // "/follow Bob" is when Bob is already selected.
                [&] {
                    std::string n = gh->lookupName(gh->getTargetGuid());
                    toLowerInPlace(n);
                    return !n.empty() && n == who;
                }();
            if (meansTarget && gh->getTargetGuid() != 0) gh->followTarget();
            return 0;
        }},
                // Inspect and duel, both of which unitpopup.lua calls straight
                // out of the unit right-click menu - and neither was bound at
                // all, so choosing either raised. A missing global is worse
                // than a stub: an unanswered call throws where an unfired event
                // only goes unheard.
                {"InspectUnit", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* unit = luaL_optstring(L, 1, "target");
            if (!gh || !unit) return 0;
            std::string uid(unit);
            toLowerInPlace(uid);
            if (const uint64_t guid = resolveUnitGuid(gh, uid)) gh->inspectUnit(guid);
            return 0;
        }},
                // The achievement comparison the same menu offers. The query
                // goes out with the inspect above, and there is no comparison
                // window to show it in - but it is bound rather than absent so
                // that picking it does nothing instead of throwing.
                {"InspectAchievements", [](lua_State* L) -> int { (void)L; return 0; }},

                // ---- The rest of what the unit right-click menu calls ----
                //
                // UnitPopup_ShowMenu asks several of these while deciding which
                // entries to show, so a missing one does not skip an entry - it
                // throws part way through building the menu and there is no
                // menu at all. Right-clicking the player's own frame reached
                // GetPVPDesired that way.
                //
                // Everything here is either answered properly or answered
                // safely. A conservative answer hides a menu entry; a missing
                // global breaks the whole menu.
                {"GetPVPDesired", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            bool on = false;
            if (gh) {
                if (auto e = gh->getEntityManager().getEntity(gh->getPlayerGuid())) {
                    // UNIT_FIELD_FLAGS index 59, bit 0x1000 = UNIT_FLAG_PVP,
                    // read the same way CombatHandler::togglePvp reads it.
                    on = (e->getField(59) & 0x00001000u) != 0;
                }
            }
            lua_pushnumber(L, on ? 1 : 0);   // the menu compares it against 1 and 0
            return 1;
        }},
                {"SetPVP", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) return 0;
            const bool want = lua_isnumber(L, 1) ? (lua_tonumber(L, 1) != 0)
                                                 : (lua_toboolean(L, 1) != 0);
            bool on = false;
            if (auto e = gh->getEntityManager().getEntity(gh->getPlayerGuid())) {
                on = (e->getField(59) & 0x00001000u) != 0;
            }
            // The client can only toggle, so only toggle when it would land on
            // what was asked for - otherwise SetPVP(1) while already flagged
            // would turn it off.
            if (want != on) gh->togglePvp();
            return 0;
        }},
                {"UnitIsInMyGuild", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* name = luaL_optstring(L, 1, "");
            if (!gh || !name || !*name) { lua_pushboolean(L, 0); return 1; }
            bool found = false;
            for (const auto& m : gh->getGuildRoster().members) {
                if (m.name == name) { found = true; break; }
            }
            lua_pushboolean(L, found);
            return 1;
        }},
                // Answered no rather than left missing. Each hides one menu
                // entry, which is what should happen for something this client
                // cannot do - party promotion and demotion, channel moderation,
                // granting levels, reporting chat, and the recruit-a-friend and
                // Battle.net pieces have no client support behind them.
                // A pet can be named once, after the first tame, and the
                // client has tracked that all along - PET_RENAMEABLE is fired
                // and this answered no regardless, so the unit menu never
                // offered it.
                {"PetCanBeRenamed", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushboolean(L, gh && gh->isPetRenameable() ? 1 : 0);
            return 1;
        }},
                // The other end: PETRENAMECONFIRM's accept calls this, so the
                // naming dialog appeared, took a name, asked for confirmation
                // and then raised on an unbound global. The packet has existed
                // the whole time.
                {"PetRename", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* name = luaL_optstring(L, 1, "");
            if (gh && name && *name) gh->renamePet(name);
            return 0;
        }},
                {"CanGrantLevel",             [](lua_State* L) -> int { lua_pushboolean(L, 0); return 1; }},
                // Whether that line can be reported: it has to be one this
                // client still remembers the sender of, and not one of the
                // player's own.
                {"CanComplainChat", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const uint32_t lineId = static_cast<uint32_t>(luaL_optnumber(L, 1, 0));
            const uint64_t guid = (gh && lineId) ? gh->chatLineSender(lineId) : 0;
            lua_pushboolean(L, (guid != 0 && guid != gh->getPlayerGuid()) ? 1 : 0);
            return 1;
        }},
                {"CanChangePlayerDifficulty", [](lua_State* L) -> int { lua_pushboolean(L, 0); return 1; }},
                {"IsSilenced",                [](lua_State* L) -> int { lua_pushboolean(L, 0); return 1; }},
                {"IsDisplayChannelModerator", [](lua_State* L) -> int { lua_pushboolean(L, 0); return 1; }},
                // PromoteToLeader(name) - /promote, /pr, and the unit menu's
                // "Promote to Leader". The client had no outgoing side for this
                // at all: SMSG_GROUP_SET_LEADER was handled and CMSG was never
                // sent, so leadership could be received and never given.
                {"PromoteToLeader", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            std::string name(luaL_optstring(L, 1, ""));
            if (!gh || name.empty()) return 0;
            toLowerInPlace(name);
            for (const auto& m : gh->getPartyData().members) {
                std::string n = m.name;
                toLowerInPlace(n);
                if (n == name) { gh->promoteToLeader(m.guid); return 0; }
            }
            return 0;
        }},
                // Raid assistant, given and taken. The unit menu passes a
                // *name* rather than a unit token - PromoteToAssistant(fullname,
                // 1) - and the group roster is the one place a name can be
                // turned into a guid here, which is also the only place it needs
                // to be: assistant is a thing only a group member can hold.
                //
                // Both were no-ops while the opcode sat in all three expansion
                // maps, so the entry appeared in the menu and did nothing.
                {"PromoteToAssistant", [](lua_State* L) -> int {
            setGroupAssistantByName(L, true);
            return 0;
        }},
                {"DemoteAssistant", [](lua_State* L) -> int {
            setGroupAssistantByName(L, false);
            return 0;
        }},
                {"GrantLevel",                [](lua_State* L) -> int { (void)L; return 0; }},
                // Whether the player passes on every loot roll. The setter
                // accepted and forgot, and the getter beside it answered a
                // flat false - so the unit menu's "Opt out of loot" entry
                // never showed a tick and never turned itself off again.
                {"SetOptOutOfLoot", [](lua_State* L) -> int {
            const bool out = lua_toboolean(L, 1) != 0;
            optOutOfLoot() = out;
            if (auto* gh = getGameHandler(L)) gh->sendOptOutOfLoot(out);
            return 0;
        }},
                {"GetOptOutOfLoot", [](lua_State* L) -> int {
            lua_pushboolean(L, optOutOfLoot() ? 1 : 0);
            return 1;
        }},
                // Both send a real opcode now; sendSetDifficulty used to write
                // CMSG_CHANGEPLAYER_DIFFICULTY, which the server reads off the
                // wire and throws away.
                // The menu row, which counts from one, less one for the wire,
                // which counts from zero. Sent as it came, picking Normal set
                // Heroic, and picking Heroic sent 2 - which the server tests
                // against MAX_DUNGEON_DIFFICULTY and drops, so it did nothing.
                {"SetDungeonDifficulty", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int row = static_cast<int>(luaL_optnumber(L, 1, 1));
            if (gh && row >= 1) gh->sendSetDifficulty(static_cast<uint32_t>(row - 1), /*raid=*/false);
            return 0;
        }},
                {"SetRaidDifficulty", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int row = static_cast<int>(luaL_optnumber(L, 1, 1));
            if (gh && row >= 1) gh->sendSetDifficulty(static_cast<uint32_t>(row - 1), /*raid=*/true);
            return 0;
        }},
                                {"ChannelBan", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* ch = luaL_optstring(L, 1, "");
            const char* who = luaL_optstring(L, 2, "");
            if (gh && ch && who && *ch && *who)
                gh->channelModeration(game::Opcode::CMSG_CHANNEL_BAN, ch, who);
            return 0;
        }},
                                {"ChannelKick", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* ch = luaL_optstring(L, 1, "");
            const char* who = luaL_optstring(L, 2, "");
            if (gh && ch && who && *ch && *who)
                gh->channelModeration(game::Opcode::CMSG_CHANNEL_KICK, ch, who);
            return 0;
        }},
                                {"ChannelModerator", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* ch = luaL_optstring(L, 1, "");
            const char* who = luaL_optstring(L, 2, "");
            if (gh && ch && who && *ch && *who)
                gh->channelModeration(game::Opcode::CMSG_CHANNEL_MODERATOR, ch, who);
            return 0;
        }},
                                {"ChannelUnmoderator", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* ch = luaL_optstring(L, 1, "");
            const char* who = luaL_optstring(L, 2, "");
            if (gh && ch && who && *ch && *who)
                gh->channelModeration(game::Opcode::CMSG_CHANNEL_UNMODERATOR, ch, who);
            return 0;
        }},
                // SetChannelOwner(channel, name) - the same two strings the
                // eight moderation commands send, on an opcode that was mapped
                // and never built.
                {"SetChannelOwner", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* ch = luaL_optstring(L, 1, "");
            const char* who = luaL_optstring(L, 2, "");
            if (gh && ch && who && *ch && *who)
                gh->channelModeration(game::Opcode::CMSG_CHANNEL_SET_OWNER, ch, who);
            return 0;
        }},
                {"BNSetToonBlocked",          [](lua_State* L) -> int { (void)L; return 0; }},

                // ---- What the summon and resurrect popups read ----
                //
                // These two prompts were handed to FrameXML so they would stop
                // being asked twice, and that put their text and timer
                // functions on a live path for the first time. Both were
                // missing, so the summon popup threw in its OnShow and
                // ShowResurrectRequest threw before it could show anything -
                // the handover fixed a duplicate and replaced it with a raise.
                {"GetSummonConfirmSummoner", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushstring(L, gh ? gh->getSummonerName().c_str() : "");
            return 1;
        }},
                {"GetSummonConfirmTimeLeft", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushnumber(L, gh ? gh->getSummonTimeoutSec() : 0.0f);
            return 1;
        }},
                // The destination is not carried in what this client parses,
                // and the popup prints it into a sentence - so an empty string
                // rather than nil, which would concatenate into a raise.
                {"GetSummonConfirmAreaName", [](lua_State* L) -> int {
            lua_pushstring(L, ""); return 1;
        }},
                // Asked every frame by the popup's OnUpdate to grey the accept
                // button. The server refuses a summon it will not honour, so
                // the client does not second-guess it.
                {"PlayerCanTeleport", [](lua_State* L) -> int {
            lua_pushboolean(L, 1); return 1;
        }},
                // Which of three resurrect popups to raise. Both false picks
                // RESURRECT_NO_TIMER, the plain "accept?" prompt, which is the
                // right one for a player casting a resurrect - sickness and the
                // release timer belong to the spirit healer path, which this
                // client answers itself.
                // Which of the three resurrect dialogs is raised. Both
                // answered false, so ShowResurrectRequest always took the last
                // branch and offered the no-timer one - the variant that does
                // not warn about sickness and does not run out. The flag that
                // decides has been on the wire the whole time, four bytes past
                // where this client was reading.
                {"ResurrectHasSickness", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushboolean(L, (gh && gh->resurrectHasSickness()) ? 1 : 0);
            return 1;
        }},
                {"ResurrectHasTimer", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushboolean(L, (gh && gh->resurrectHasTimer()) ? 1 : 0);
            return 1;
        }},
                // StaticPopup_Show calls this on *every* popup, before it does
                // anything else: `if ( InCinematic() and not
                // info.interruptCinematic )`. It was unbound, so every popup
                // raised there and none of them appeared - and handing the
                // confirmation prompts to FrameXML is what put that call on a
                // live path. The resurrect dialog was the visible half; the
                // group invite, the summon, the talent wipe and the delete-item
                // confirmation were all going the same way.
                //
                // False because this client plays no cinematics, and there is
                // no state to read.
                // ---- The death and release popup ----
                //
                // StaticPopupDialogs["DEATH"] is raised every time the player
                // dies, and its OnShow calls HasSoulstone while its OnAccept
                // calls RepopMe and CannotBeResurrected. All three were
                // unbound, so the release popup raised in OnShow and never
                // appeared - and releasing spirit from it was impossible even
                // if it had.
                {"RepopMe", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L)) gh->releaseSpirit();
            return 0;
        }},
                // The text for the second button, or nil for no button at all.
                // This client knows the same thing as canSelfRes: the server
                // sent SMSG_PRE_RESURRECT because a Reincarnation or Twisting
                // Nether is up.
                {"HasSoulstone", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (gh && gh->canSelfRes()) { lua_pushstring(L, "Use Soulstone"); return 1; }
            lua_pushnil(L);
            return 1;
        }},
                {"UseSoulstone", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L)) gh->useSelfRes();
            return 0;
        }},
                // False: the corpse is always recoverable here. Answering true
                // would make the death popup close itself.
                {"CannotBeResurrected", [](lua_State* L) -> int {
            lua_pushboolean(L, 0); return 1;
        }},
                // IsActiveBattlefieldArena is not here. lua_system_api has the
                // one that answers it - whether the current map is an arena,
                // in the two values the caller unpacks - and registers after
                // this file, so it was already the one in use. This copy was a
                // flat false in a single value, dead and one value short of
                // the contract if it had ever won.
                // Seconds before the corpse is released automatically. -1 is
                // WoW's "no timer", which is the branch that prints the release
                // prompt without a countdown - right for a client that does not
                // run one.
                {"GetReleaseTimeRemaining", [](lua_State* L) -> int {
            lua_pushnumber(L, -1); return 1;
        }},
                // Asked from an OnUpdate, so this runs on every frame the
                // popup is up rather than once.
                //
                // IsOutOfBounds decides whether the death popup shows the
                // falling-to-your-death variant, and nothing here says which
                // it is.
                {"IsOutOfBounds", [](lua_State* L) -> int {
            lua_pushboolean(L, 0); return 1;
        }},
                {"CheckTalentMasterDist", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushboolean(L,
                gh && npcWithinInteractRange(gh, gh->getTalentWipeNpcGuid()) ? 1 : 0);
            return 1;
        }},
                // ---- Reached through FrameXML's own calls, not directly ----
                //
                // Found by walking outward from each unbound name to the live
                // file that reaches it, rather than judging by the file the
                // call is written in. That distinction had already cost two
                // live raises: GetQuestGreenRange is written in uiparent.lua
                // and reached from the target frame, GetBindingByKey the same
                // way from static popups.
                //
                // The Battle.net set arrives through the unit menu:
                // UnitPopup_OnUpdate asks CanCooperateWithToon, which calls
                // BNGetToonInfo with nothing guarding it. Answering nothing
                // makes the comparison that follows false, which is the same
                // outcome as having no Battle.net friends.
                {"BNGetToonInfo",               [](lua_State* L) -> int { (void)L; return 0; }},
                {"BNGetFriendInfo",             [](lua_State* L) -> int { (void)L; return 0; }},
                {"BNGetFriendInfoByID",         [](lua_State* L) -> int { (void)L; return 0; }},
                {"BNGetFriendInviteInfo",       [](lua_State* L) -> int { (void)L; return 0; }},
                {"BNGetNumConversationMembers", [](lua_State* L) -> int { lua_pushnumber(L, 0); return 1; }},
                {"BNGetConversationMemberInfo", [](lua_State* L) -> int { (void)L; return 0; }},
                {"BNRequestFOFInfo",            [](lua_State* L) -> int { (void)L; return 0; }},
                // The battleground queue timer, read in a loop from the
                // minimap's battlefield icon while it is shown.
                {"GetBattlefieldPortExpiration", [](lua_State* L) -> int {
            lua_pushnumber(L, 0); return 1; }},
                // The innkeeper's prompt: CONFIRM_BINDER's accept sends the
                // reply, and its OnUpdate hides the popup when the player walks
                // away. Distance to the innkeeper is not tracked, and staying
                // up is the lesser wrong - the server refuses a bind from too
                // far off anyway.
                {"ConfirmBinder", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L)) gh->confirmBinder();
            return 0;
        }},
                // Both are asked from the confirmation's own OnUpdate and both
                // used to answer a flat yes, so neither dialog ever closed
                // itself: walking away from the innkeeper left the bind
                // confirmation on screen, and the server refuses the request
                // that button then sends.
                //
                // Five yards is what the server means by interacting -
                // AzerothCore's INTERACTION_DISTANCE - measured against the
                // npc each dialog names. A dialog whose npc has gone out of
                // sight has no entity to measure, and that answers no for the
                // same reason: it is no longer there to talk to.
                {"CheckBinderDist", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushboolean(L, gh && npcWithinInteractRange(gh, gh->getBinderGuid()) ? 1 : 0);
            return 1;
        }},
                {"InCinematic", [](lua_State* L) -> int {
            lua_pushboolean(L, 0); return 1;
        }},
                // The other half of RESURRECT_NO_TIMER's OnCancel:
                // `UnitIsDead("player") and not UnitIsControlling("player")`.
                // Controlling means driving a vehicle or a possessed creature,
                // neither of which this client has.
                {"UnitIsControlling", [](lua_State* L) -> int {
            lua_pushboolean(L, 0); return 1;
        }},

                // ---- What chatframe.lua's slash commands call ----
                //
                // Forty-eight names in that file were unbound, and unlike the
                // Battle.net set they sit behind no feature flag: each is a
                // slash command handler, so each is a raise the moment someone
                // types that command. That is what kept "chat" out of the
                // candidate list, and binding them is what lets it back in.
                //
                // Implemented where this client can do the thing, answered
                // safely where it cannot. A safe answer makes the command do
                // nothing, which is the honest outcome for a feature that is
                // not there; a missing name takes the chat frame down with it.

                // The conditional parser every secure slash command runs its
                // argument through: "/cast [mod:shift] A; B". With no
                // conditional support the whole argument is the action and
                // there is no target clause, which is what an unconditional
                // command means anyway.
                {"SecureCmdOptionParse", [](lua_State* L) -> int {
            const char* msg = luaL_optstring(L, 1, "");
            lua_pushstring(L, msg ? msg : "");
            lua_pushnil(L);
            return 2;
        }},
                {"GuildInfo", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L)) gh->requestGuildInfo();
            return 0;
        }},
                {"GuildSetLeader", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* name = luaL_optstring(L, 1, "");
            if (gh && name && *name) gh->setGuildLeader(name);
            return 0;
        }},
                // Not logging, and there is nothing to turn on - false is the
                // true answer rather than a placeholder.
                {"LoggingChat",   [](lua_State* L) -> int { lua_pushboolean(L, 0); return 1; }},
                {"LoggingCombat", [](lua_State* L) -> int { lua_pushboolean(L, 0); return 1; }},
                {"BNIsFriend",      [](lua_State* L) -> int { lua_pushboolean(L, 0); return 1; }},
                {"BNIsToonBlocked", [](lua_State* L) -> int { lua_pushboolean(L, 0); return 1; }},
                {"BNGetNumFriendInvites", [](lua_State* L) -> int { lua_pushnumber(L, 0); return 1; }},
                {"GetClickFrame",         [](lua_State* L) -> int { lua_pushnil(L); return 1; }},
                // Pet autocast is left alone deliberately rather than wired to
                // togglePetSpellAutocast: these two say enable and disable, the
                // client can only toggle, and nothing reads back which state a
                // pet spell is in - so "enable" on an already-autocasting spell
                // would turn it off. Doing nothing beats doing the opposite.
                // The three pet-autocast slash commands. They pass the spell by
                // *name*, since that is what the player typed and what
                // SecureCmdOptionParse hands back - the pet bar's own toggle
                // takes a slot and is bound separately.
                //
                // Only sent when the state would actually change: the wire has
                // a toggle and nothing else, so asking to enable something
                // already on would turn it off, which is the opposite of what
                // was typed.
                {"EnableSpellAutocast",  [](lua_State* L) -> int {
            return setPetAutocastByName(L, true);
        }},
                {"DisableSpellAutocast", [](lua_State* L) -> int {
            return setPetAutocastByName(L, false);
        }},
                // ToggleSpellAutocast(spell, bookType) - asked in two shapes.
                //
                // The spellbook passes a book *slot* and the book it belongs
                // to; /petautocasttoggle passes a name. Only the name was
                // handled, so right-clicking a pet spell in the spellbook
                // looked up a pet spell called "12" and toggled nothing at all.
                {"ToggleSpellAutocast",  [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const uint32_t id = (!lua_isnoneornil(L, 2) && lua_isnumber(L, 1))
                ? spellIdForCall(L, gh)
                : petSpellByName(gh, luaL_optstring(L, 1, ""));
            if (gh && id != 0) gh->togglePetSpellAutocast(id);
            return 0;
        }},
                // The four chat-window list editors moved to
                // lua_system_api.cpp, beside the chat window store they write
                // to. Two registrations of one name would be settled by load
                // order, so they are not also here.
                                // The eight channel moderation commands. Every one is the
                // same packet - channel name then player name - and none of
                // them was ever sent: the opcodes existed and nothing built
                // them, so /cinvite and the channel entries on the unit menu
                // reached bindings with nothing behind them.
                {"ChannelInvite", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* ch = luaL_optstring(L, 1, "");
            const char* who = luaL_optstring(L, 2, "");
            if (gh && ch && who && *ch && *who)
                gh->channelModeration(game::Opcode::CMSG_CHANNEL_INVITE, ch, who);
            return 0;
        }},
                                {"ChannelMute", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* ch = luaL_optstring(L, 1, "");
            const char* who = luaL_optstring(L, 2, "");
            if (gh && ch && who && *ch && *who)
                gh->channelModeration(game::Opcode::CMSG_CHANNEL_MUTE, ch, who);
            return 0;
        }},
                                {"ChannelUnmute", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* ch = luaL_optstring(L, 1, "");
            const char* who = luaL_optstring(L, 2, "");
            if (gh && ch && who && *ch && *who)
                gh->channelModeration(game::Opcode::CMSG_CHANNEL_UNMUTE, ch, who);
            return 0;
        }},
                                {"ChannelUnban", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* ch = luaL_optstring(L, 1, "");
            const char* who = luaL_optstring(L, 2, "");
            if (gh && ch && who && *ch && *who)
                gh->channelModeration(game::Opcode::CMSG_CHANNEL_UNBAN, ch, who);
            return 0;
        }},
                {"ChannelToggleAnnouncements", [](lua_State* L) -> int { (void)L; return 0; }},
                {"DisplayChannelOwner",      [](lua_State* L) -> int { (void)L; return 0; }},
                {"ListChannels", [](lua_State* L) -> int {
                    // /chatlist. The client knows exactly this - which channels
                    // it is in and what number each one has - and answered
                    // nothing, so the command printed nothing and read as a
                    // command that does not exist.
                    //
                    // Members are a separate question this client cannot answer,
                    // which is why ListChannelByName is still inert: it lists who
                    // is in one, and the server is never asked.
                    auto* gh = getGameHandler(L);
                    if (!gh) return 0;
                    const auto& joined = gh->getJoinedChannels();
                    if (joined.empty()) {
                        gh->addSystemChatMessage("You are not in any channels.");
                        return 0;
                    }
                    for (size_t i = 0; i < joined.size(); ++i) {
                        gh->addSystemChatMessage(
                            std::to_string(i + 1) + ". " + joined[i]);
                    }
                    return 0;
                }},
                {"ListChannelByName",        [](lua_State* L) -> int { (void)L; return 0; }},
                // SetChannelPassword(channel, password) - the same shape again,
                // and the one place an empty second string is meant: sending
                // no password is how a channel's password is taken off.
                {"SetChannelPassword", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* ch = luaL_optstring(L, 1, "");
            const char* pass = luaL_optstring(L, 2, "");
            if (gh && ch && *ch)
                gh->channelModeration(game::Opcode::CMSG_CHANNEL_PASSWORD, ch,
                                      pass ? pass : "", /*allowEmptyTarget=*/true);
            return 0;
        }},
                // The four arena team commands. Each is named by a static
                // popup's button and by a slash command - /teaminvite,
                // /teamquit, /teamkick, /teamcaptain - and all four did
                // nothing: the popup closed, the command was accepted, and no
                // packet was sent. The opcodes have been in the maps the whole
                // time with nothing building them.
                //
                // The first argument is the team's position in the list, which
                // is how the interface counts the three bracket sizes; the wire
                // is addressed by the team's own id.
                {"ArenaTeamInviteByName", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const uint32_t id = arenaTeamIdAtIndex(gh, static_cast<int>(luaL_optnumber(L, 1, 0)));
            const char* name = luaL_optstring(L, 2, "");
            if (gh && id) gh->arenaTeamInvite(id, name ? name : "");
            return 0;
        }},
                {"ArenaTeamLeave", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const uint32_t id = arenaTeamIdAtIndex(gh, static_cast<int>(luaL_optnumber(L, 1, 0)));
            if (gh && id) gh->arenaTeamLeave(id);
            return 0;
        }},
                {"ArenaTeamSetLeaderByName", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const uint32_t id = arenaTeamIdAtIndex(gh, static_cast<int>(luaL_optnumber(L, 1, 0)));
            const char* name = luaL_optstring(L, 2, "");
            if (gh && id) gh->arenaTeamSetLeader(id, name ? name : "");
            return 0;
        }},
                {"ArenaTeamUninviteByName", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const uint32_t id = arenaTeamIdAtIndex(gh, static_cast<int>(luaL_optnumber(L, 1, 0)));
            const char* name = luaL_optstring(L, 2, "");
            if (gh && id) gh->arenaTeamRemove(id, name ? name : "");
            return 0;
        }},
                {"BNGetConversationInfo",    [](lua_State* L) -> int { (void)L; return 0; }},
                {"BNInviteToConversation",   [](lua_State* L) -> int { (void)L; return 0; }},
                {"BNLeaveConversation",      [](lua_State* L) -> int { (void)L; return 0; }},
                {"BNListConversation",       [](lua_State* L) -> int { (void)L; return 0; }},
                {"BNSendConversationMessage",[](lua_State* L) -> int { (void)L; return 0; }},
                {"BNSendWhisper",            [](lua_State* L) -> int { (void)L; return 0; }},
                // ConsoleExec(command) - the console, of which one command is
                // reachable and worth honouring.
                //
                // SlashCmdList["RELOAD"] is ConsoleExec("reloadui") and nothing
                // else, and this client's chat tries SlashCmdList before its
                // own registry - so /reload, /reloadui and /rl all landed here
                // and did nothing while the client's own reload sat behind
                // them. Routed to the same request ReloadUI makes.
                {"ConsoleExec", [](lua_State* L) -> int {
            std::string cmd(luaL_optstring(L, 1, ""));
            toLowerInPlace(cmd);
            if (cmd == "reloadui") {
                if (auto* svc = getLuaServices(L); svc && svc->requestReloadUI) {
                    svc->requestReloadUI();
                }
            }
            return 0;
        }},
                // The two buttons under the AddOns list. EnableAddOn and
                // DisableAddOn have driven the real setting for a while; these
                // two were left discarding the click, so "Disable All" looked
                // like a button that does not work. They are the same
                // instruction, once per addon.
                //
                // The character argument is ignored for the same reason
                // EnableAddOn ignores it: this client keeps one enabled list,
                // not one per character.
                {"DisableAllAddOns",         [](lua_State* L) -> int {
                    return setAllAddOnsEnabled(L, false);
                }},
                {"EnableAllAddOns",          [](lua_State* L) -> int {
                    return setAllAddOnsEnabled(L, true);
                }},
                {"SetTaxiBenchmarkMode",     [](lua_State* L) -> int { (void)L; return 0; }},
                {"StopMacro",                [](lua_State* L) -> int { (void)L; return 0; }},
                // The targeting variants this client has no equivalent for.
                // targetEnemy and targetFriend do not filter to players, and
                // one last-target is tracked rather than one per side - so
                // wiring these to the nearest thing would target the wrong
                // unit, which is worse than not answering the command.
                {"TargetLastEnemy", [](lua_State* L) -> int {
                    if (auto* gh = getGameHandler(L)) gh->targetLastEnemy();
                    return 0;
                }},
                {"TargetLastFriend", [](lua_State* L) -> int {
                    if (auto* gh = getGameHandler(L)) gh->targetLastFriend();
                    return 0;
                }},
                {"TargetNearestEnemyPlayer", [](lua_State* L) -> int {
                    if (auto* gh = getGameHandler(L)) gh->targetNearestEnemyPlayer(wantsReverseArg(L));
                    return 0;
                }},
                {"TargetNearestFriendPlayer", [](lua_State* L) -> int {
                    if (auto* gh = getGameHandler(L)) gh->targetNearestFriendPlayer(wantsReverseArg(L));
                    return 0;
                }},
                {"TargetNearestPartyMember", [](lua_State* L) -> int {
                    if (auto* gh = getGameHandler(L)) gh->targetNearestPartyMember(wantsReverseArg(L));
                    return 0;
                }},
                {"TargetNearestRaidMember", [](lua_State* L) -> int {
                    if (auto* gh = getGameHandler(L)) gh->targetNearestRaidMember(wantsReverseArg(L));
                    return 0;
                }},
                {"StartDuel", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* unit = luaL_optstring(L, 1, "target");
            if (!gh || !unit) return 0;
            std::string uid(unit);
            toLowerInPlace(uid);
            if (const uint64_t guid = resolveUnitGuid(gh, uid)) gh->proposeDuel(guid);
            return 0;
        }},
                {"RandomRoll", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            int mn = static_cast<int>(luaL_optnumber(L, 1, 1));
            int mx = static_cast<int>(luaL_optnumber(L, 2, 100));
            if (gh) gh->randomRoll(mn, mx);
            return 0;
        }},
                {"JoinChannelByName", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* name = luaL_checkstring(L, 1);
            const char* pw = luaL_optstring(L, 2, "");
            if (gh) gh->joinChannel(name, pw);
            return 0;
        }},
                {"LeaveChannelByName", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* name = luaL_checkstring(L, 1);
            if (gh) gh->leaveChannel(name);
            return 0;
        }},
                // GetChannelName(indexOrName) → id, name, instanceID
                //
                // Three values, and the first is a NUMBER. This was returning
                // the seven-value shape GetChannelDisplayInfo uses, so the
                // channel number came back as the channel's name and
                // chatframe.lua compared a string to a number -
                // `if ( channelNum <= 0 )` - which raises rather than failing
                // quietly. Addressing a channel by number, "/1 hello", went
                // through exactly that line.
                //
                // Zero means "not in that channel", which is what every caller
                // tests for.
                {"GetChannelName", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) { lua_pushnumber(L, 0); lua_pushnil(L); lua_pushnumber(L, 0); return 3; }
            const auto& joined = gh->getJoinedChannels();

            int index = 0;
            if (lua_isnumber(L, 1)) {
                index = static_cast<int>(lua_tonumber(L, 1));
            } else if (const char* wanted = lua_tostring(L, 1)) {
                // Accepts a name as well as an index; chat's slash handlers
                // pass whichever the player typed.
                for (size_t i = 0; i < joined.size(); ++i) {
                    if (joined[i] == wanted) { index = static_cast<int>(i) + 1; break; }
                }
            }
            if (index < 1 || index > static_cast<int>(joined.size())) {
                lua_pushnumber(L, 0);
                lua_pushnil(L);
                lua_pushnumber(L, 0);
                return 3;
            }
            lua_pushnumber(L, index);
            lua_pushstring(L, joined[static_cast<size_t>(index) - 1].c_str());
            lua_pushnumber(L, 0);   // instanceID - one instance of each here
            return 3;
        }},
    };
    for (const auto& [name, func] : api) {
        lua_pushcfunction(L, func);
        lua_setglobal(L, name);
    }
}

} // namespace wowee::addons
