// lua_lfg_api.cpp - the dungeon finder.
//
// Split out rather than added to a neighbour because it is a whole panel's
// worth of surface: lfdframe.lua, lfgframe.lua and lfrframe.lua between them
// called forty-one globals and not one of them was bound. The LFD micro button
// sits on the main bar, which is handed over by default, so clicking it raised
// on the first line of LFDParentFrame's OnShow.
//
// What is real here and what is not:
//
//   * The dungeon list is real, read from LFGDungeons.dbc through
//     GameHandler::getLfgDungeons(). Names, levels, groups, difficulty and
//     faction all come from the file.
//   * The queue is real - joining, leaving, roles and the queued list go
//     through the LFG verbs this client has had all along.
//   * Locks and rewards are real, off SMSG_LFG_PLAYER_INFO.
//   * The boot vote is real, including who is being voted on and whether this
//     player has answered - the victim arrives as a guid, not a name.
//   * The role check knows which dungeons it is for and how many are
//     answering it.
//   * The proposal's per-member detail, party backfill and the raid browser's
//     search are NOT modelled. They answer empty, and each says why where it
//     is bound.
//
// That list is worth keeping honest. It read "locks, rewards, the boot vote's
// detail ... are NOT modelled" for some time after all three had been written,
// and a note like this is what the next pass reads instead of the code.
//
// An empty answer is chosen deliberately over a plausible one. Every caller
// here guards, and a fabricated reward or lock reads as fact.
#include "addons/lua_api_helpers.hpp"
#include "addons/lua_engine.hpp"
#include "game/game_utils.hpp"

#include <algorithm>
#include <string>
#include <unordered_map>
#include <vector>

namespace wowee::addons {

namespace {

/// FrameXML numbers a category header with the negative of its group id, and
/// keeps headers and dungeons in one key space - LFGIsIDHeader is `id < 0`,
/// while LFGListUpdateHeaderEnabledAndLockedStates indexes the same lists by
/// the groupID it read out of the info table. So the groupID reported for a
/// dungeon has to be the header's id, not the raw number from the DBC.
int headerIdFor(uint32_t groupId) { return -static_cast<int>(groupId); }

/// LFGDungeons.dbc has no name for a category. The real client carries these
/// itself; they are the standard names for the groups the file actually uses,
/// and group 10 is absent from the data rather than omitted here.
const char* groupName(uint32_t groupId) {
    switch (groupId) {
        case 1:  return "Classic Dungeons";
        case 2:  return "Burning Crusade Dungeons";
        case 3:  return "Burning Crusade Heroic Dungeons";
        case 4:  return "Wrath of the Lich King Dungeons";
        case 5:  return "Wrath of the Lich King Heroic Dungeons";
        case 6:  return "Classic Raids";
        case 7:  return "Burning Crusade Raids";
        case 8:  return "Wrath of the Lich King Raids (10)";
        case 9:  return "Wrath of the Lich King Raids (25)";
        case 11: return "Holiday Dungeons";
        default: return "Dungeons";
    }
}

/// How many the group is built for. The file does not say, so it comes from
/// what the group is: five for a dungeon, and the two Wrath raid groups are
/// split ten from twenty-five, which is the whole reason they are two groups.
int maxPlayersFor(const game::LfgDungeon& d) {
    if (d.typeId == static_cast<uint32_t>(game::LfgTypeId::Raid)) {
        return d.groupId == 9 ? 25 : 10;
    }
    return 5;
}

/// The dungeon finder lists dungeons and the raid browser lists raids, so each
/// asks for its own half of the same file.
bool isDungeonSide(const game::LfgDungeon& d) {
    return d.typeId == static_cast<uint32_t>(game::LfgTypeId::Dungeon) ||
           d.typeId == static_cast<uint32_t>(game::LfgTypeId::Heroic) ||
           d.typeId == static_cast<uint32_t>(game::LfgTypeId::Random);
}
bool isRaidSide(const game::LfgDungeon& d) {
    return d.typeId == static_cast<uint32_t>(game::LfgTypeId::Raid);
}

/// Which dungeons the player has ticked, and which headers are folded up.
/// Both are the panel's own state - FrameXML says so in a comment, "we
/// maintain this list in Lua" - and the only reason they are here is that it
/// asks the client for the starting value.
std::unordered_map<int, bool>& enabledDungeons() {
    static std::unordered_map<int, bool> enabled;
    return enabled;
}
std::unordered_map<int, bool>& collapsedHeaders() {
    static std::unordered_map<int, bool> collapsed;
    return collapsed;
}

int luaReturnTrue(lua_State* L)    { lua_pushboolean(L, 1); return 1; }
int luaReturnNothing(lua_State* L) { (void)L; return 0; }

/// The roles the player has offered. Kept on the handler now - the ready
/// dialog names the role it found you a group for, and a static in this file
/// was not somewhere GetLFGProposal could reach.
uint8_t offeredRoles(game::GameHandler* gh) {
    const uint8_t roles = gh ? gh->getLfgOfferedRoles() : 0;
    return roles ? roles : 0x01;   // "player" is always set
}

/// Build the ordered id list one side of the panel shows: a header, then the
/// dungeons under it, then the next header.
void pushChoiceOrder(lua_State* L, game::GameHandler* gh, bool dungeonSide) {
    lua_newtable(L);
    if (!gh) return;
    int n = 0;
    uint32_t lastGroup = 0;
    for (const auto& d : gh->getLfgDungeons()) {
        // groupId 0 is a row that belongs under no header - the old
        // per-zone entries - and LFDList_DefaultFilterFunction drops them
        // anyway. Leaving them out here keeps the list honest either way.
        if (d.groupId == 0) continue;
        if (dungeonSide ? !isDungeonSide(d) : !isRaidSide(d)) continue;
        if (d.groupId != lastGroup) {
            lastGroup = d.groupId;
            lua_pushinteger(L, headerIdFor(d.groupId));
            lua_rawseti(L, -2, ++n);
        }
        lua_pushinteger(L, static_cast<lua_Integer>(d.id));
        lua_rawseti(L, -2, ++n);
    }
}

/// One row of the info table: the twelve values LFG_RETURN_VALUES names, in
/// its order. Getting this order wrong is invisible - every read is by index
/// through that table - so it is written out here field by field.
void pushInfoRow(lua_State* L, const char* name, int typeId, int minLevel,
                 int maxLevel, int recLevel, int minRecLevel, int maxRecLevel,
                 int expansion, int groupId, const char* texture,
                 int difficulty, int maxPlayers) {
    lua_newtable(L);
    auto set = [&](int idx, auto push) { push(); lua_rawseti(L, -2, idx); };
    set(1,  [&]{ lua_pushstring(L, name); });
    set(2,  [&]{ lua_pushinteger(L, typeId); });
    set(3,  [&]{ lua_pushinteger(L, minLevel); });
    set(4,  [&]{ lua_pushinteger(L, maxLevel); });
    set(5,  [&]{ lua_pushinteger(L, recLevel); });
    set(6,  [&]{ lua_pushinteger(L, minRecLevel); });
    set(7,  [&]{ lua_pushinteger(L, maxRecLevel); });
    set(8,  [&]{ lua_pushinteger(L, expansion); });
    set(9,  [&]{ lua_pushinteger(L, groupId); });
    set(10, [&]{ lua_pushstring(L, texture); });
    set(11, [&]{ lua_pushinteger(L, difficulty); });
    set(12, [&]{ lua_pushinteger(L, maxPlayers); });
}

}  // namespace

void registerLfgLuaAPI(lua_State* L) {
    static const std::pair<const char*, lua_CFunction> api[] = {

    // ---- The catalogue ----

    // GetLFDChoiceInfo(t) / GetLFRChoiceInfo - every listable row keyed by id,
    // headers included. FrameXML calls this once and keeps the result: "this
    // will never change (without a patch)".
    {"GetLFDChoiceInfo", [](lua_State* L) -> int {
        auto* gh = getGameHandler(L);
        lua_newtable(L);
        if (!gh) return 1;
        std::vector<uint32_t> groupsSeen;
        for (const auto& d : gh->getLfgDungeons()) {
            if (d.groupId == 0) continue;
            if (std::find(groupsSeen.begin(), groupsSeen.end(), d.groupId) == groupsSeen.end()) {
                groupsSeen.push_back(d.groupId);
                // The header's own row. Its groupID is itself, which is how
                // LFDList_SetHeaderCollapsed matches children to the header
                // that was clicked.
                pushInfoRow(L, groupName(d.groupId), 0, 0, 0, 0, 0, 0,
                            static_cast<int>(d.expansion), headerIdFor(d.groupId),
                            "", 0, 0);
                lua_rawseti(L, -2, headerIdFor(d.groupId));
            }
            pushInfoRow(L, d.name.c_str(), static_cast<int>(d.typeId),
                        static_cast<int>(d.minLevel), static_cast<int>(d.maxLevel),
                        static_cast<int>(d.recLevel), static_cast<int>(d.minRecLevel),
                        static_cast<int>(d.maxRecLevel), static_cast<int>(d.expansion),
                        headerIdFor(d.groupId), d.texture.c_str(),
                        static_cast<int>(d.difficulty), maxPlayersFor(d));
            lua_rawseti(L, -2, static_cast<lua_Integer>(d.id));
        }
        return 1;
    }},

    {"GetLFDChoiceOrder", [](lua_State* L) -> int {
        pushChoiceOrder(L, getGameHandler(L), /*dungeonSide=*/true);
        return 1;
    }},
    {"GetLFRChoiceOrder", [](lua_State* L) -> int {
        pushChoiceOrder(L, getGameHandler(L), /*dungeonSide=*/false);
        return 1;
    }},

    // ---- The three state tables the panel keeps ----
    //
    // Returned as tables keyed by id. FrameXML maintains them itself after the
    // first read; these are the starting values.

    {"GetLFDChoiceEnabledState", [](lua_State* L) -> int {
        lua_newtable(L);
        for (const auto& [id, on] : enabledDungeons()) {
            lua_pushboolean(L, on ? 1 : 0);
            lua_rawseti(L, -2, id);
        }
        return 1;
    }},
    {"GetLFDChoiceCollapseState", [](lua_State* L) -> int {
        lua_newtable(L);
        for (const auto& [id, folded] : collapsedHeaders()) {
            lua_pushboolean(L, folded ? 1 : 0);
            lua_rawseti(L, -2, id);
        }
        return 1;
    }},
    // Which dungeons the server will not let this character queue for. It
    // says so in SMSG_LFG_PLAYER_INFO, which this client skipped wholesale -
    // so every dungeon listed as queueable and the server did the refusing.
    {"GetLFDChoiceLockedState", [](lua_State* L) -> int {
        auto* gh = getGameHandler(L);
        lua_newtable(L);
        if (!gh) return 1;
        for (const auto& [dungeonId, status] : gh->getLfgLocks()) {
            if (status == 0) continue;          // 0 is not a lock
            lua_pushboolean(L, 1);
            lua_rawseti(L, -2, static_cast<lua_Integer>(dungeonId));
        }
        return 1;
    }},
    // GetLFDLockInfo(dungeonID, index) → playerName, lockedReason
    //
    // The reason is the server's lock status, which indexes
    // LFG_INSTANCE_INVALID_CODES directly - 1 expansion, 2 level too low, 3
    // too high, 4 and 5 gear, 6 raid locked. The name stays nil: the player
    // block names nobody, and the party block that would is a different packet
    // this client does not read.
    // How many players the lock list covers. One - this character. The counting
    // table answers zero for it, which was right while nothing was parsed and
    // is not now: a zero here means the loop that prints why you cannot queue
    // never runs, so the tooltip shows its heading and no reason.
    //
    // Binding it over the counting table is what that table's own comment
    // describes: a real implementation replaces the stub by existing. The
    // party's locks are a different packet this client does not read, so the
    // answer stays one rather than the group's size.
    {"GetLFDLockPlayerCount", [](lua_State* L) -> int {
        lua_pushinteger(L, 1);
        return 1;
    }},
    {"GetLFDLockInfo", [](lua_State* L) -> int {
        auto* gh = getGameHandler(L);
        const uint32_t dungeonId = static_cast<uint32_t>(luaL_optinteger(L, 1, 0));
        if (!gh || dungeonId == 0) return luaReturnNil(L);
        const auto& locks = gh->getLfgLocks();
        auto it = locks.find(dungeonId);
        if (it == locks.end()) return luaReturnNil(L);
        lua_pushnil(L);
        lua_pushinteger(L, static_cast<lua_Integer>(it->second));
        return 2;
    }},

    {"SetLFGDungeonEnabled", [](lua_State* L) -> int {
        const int id = static_cast<int>(luaL_optinteger(L, 1, 0));
        if (id != 0) enabledDungeons()[id] = lua_toboolean(L, 2) != 0;
        return 0;
    }},
    {"SetLFGHeaderCollapsed", [](lua_State* L) -> int {
        const int id = static_cast<int>(luaL_optinteger(L, 1, 0));
        if (id != 0) collapsedHeaders()[id] = lua_toboolean(L, 2) != 0;
        return 0;
    }},
    {"ClearAllLFGDungeons", [](lua_State* L) -> int {
        (void)L;
        enabledDungeons().clear();
        return 0;
    }},
    // Picking the single dungeon a "specific" queue is for.
    {"SetLFGDungeon", [](lua_State* L) -> int {
        const int id = static_cast<int>(luaL_optinteger(L, 1, 0));
        enabledDungeons().clear();
        if (id != 0) enabledDungeons()[id] = true;
        return 0;
    }},

    // ---- Roles ----

    // GetLFGRoles() → leader, tank, healer, damage. The client keeps the three
    // role flags; nobody here is a leader of an LFG group that does not exist.
    {"GetLFGRoles", [](lua_State* L) -> int {
        const uint8_t roles = offeredRoles(getGameHandler(L));
        lua_pushboolean(L, 0);
        lua_pushboolean(L, (roles & 0x02) ? 1 : 0);   // tank
        lua_pushboolean(L, (roles & 0x04) ? 1 : 0);   // healer
        lua_pushboolean(L, (roles & 0x08) ? 1 : 0);   // damage
        return 4;
    }},
    {"SetLFGRoles", [](lua_State* L) -> int {
        auto* gh = getGameHandler(L);
        if (!gh) return 0;
        // arg 1 is leader, which the server does not take here.
        uint8_t roles = 0x01;                                   // always "player"
        if (lua_toboolean(L, 2)) roles |= 0x02;
        if (lua_toboolean(L, 3)) roles |= 0x04;
        if (lua_toboolean(L, 4)) roles |= 0x08;
        gh->lfgSetRoles(roles);
        return 0;
    }},
    // Which roles this character may offer. Every class can deal damage; tank
    // and healer are gated by class rather than by spec, because this client
    // does not read the active tree here and offering too much is a queue that
    // is declined rather than a lie the player cannot see.
    {"GetAvailableRoles", [](lua_State* L) -> int {
        auto* gh = getGameHandler(L);
        const uint8_t cls = gh ? gh->getPlayerClass() : 0;
        // WARRIOR 1, PALADIN 2, PRIEST 5, SHAMAN 7, DRUID 11, DEATH KNIGHT 6
        const bool tank   = cls == 1 || cls == 2 || cls == 11 || cls == 6;
        const bool healer = cls == 2 || cls == 5 || cls == 7 || cls == 11;
        lua_pushboolean(L, tank ? 1 : 0);
        lua_pushboolean(L, healer ? 1 : 0);
        lua_pushboolean(L, 1);
        return 3;
    }},
    // The role check a party leader starts. Answering it is CMSG_LFG_SET_ROLES,
    // which SetLFGRoles above already sends, so this is the confirmation and
    // has nothing left to send.
    {"CompleteLFGRoleCheck",  luaReturnNothing},
    // GetLFGRoleUpdateSlot(index) → dungeonType, dungeonID
    //
    // The dungeons a role check is being run for. The popup names the one when
    // there is one and lists them in a tooltip when there are several; with
    // nothing to answer, every check read as "multiple dungeons".
    {"GetLFGRoleUpdateSlot", [](lua_State* L) -> int {
        auto* gh = getGameHandler(L);
        const int index = static_cast<int>(luaL_optnumber(L, 1, 0));
        if (!gh || index < 1) return luaReturnNil(L);
        const auto& dungeons = gh->getLfgRoleCheckDungeons();
        if (index > static_cast<int>(dungeons.size())) return luaReturnNil(L);
        const auto& d = dungeons[static_cast<size_t>(index) - 1];
        lua_pushnumber(L, d.typeId);
        lua_pushnumber(L, d.dungeonId);
        return 2;
    }},

    // ---- The queue ----

    {"JoinLFG", [](lua_State* L) -> int {
        auto* gh = getGameHandler(L);
        if (!gh) return 0;
        // Everything ticked, in a settled order. CMSG_LFG_JOIN carries a list
        // and the server queues for all of it, so a multi-select queue is one
        // join rather than a choice between them.
        //
        // This used to send the first entry of an unordered map and stop, which
        // is not the first dungeon the player picked - it is whichever one the
        // container happened to hand back, and a different one between runs.
        std::vector<uint32_t> chosen;
        for (const auto& [id, on] : enabledDungeons()) {
            if (on && id > 0) chosen.push_back(static_cast<uint32_t>(id));
        }
        std::sort(chosen.begin(), chosen.end());
        gh->lfgJoin(chosen, offeredRoles(gh));
        return 0;
    }},
    // The two buttons on the dungeon-ready dialog, and the same pair on the
    // minimap's queue menu. Neither was bound, so a pop that did appear could
    // not be answered - and it did not appear either, because LFG_PROPOSAL_SHOW
    // was not fired.
    {"AcceptProposal", [](lua_State* L) -> int {
        auto* gh = getGameHandler(L);
        if (gh && gh->getLfgProposalId() != 0) {
            gh->lfgAcceptProposal(gh->getLfgProposalId(), true);
        }
        return 0;
    }},
    {"RejectProposal", [](lua_State* L) -> int {
        auto* gh = getGameHandler(L);
        if (gh && gh->getLfgProposalId() != 0) {
            gh->lfgAcceptProposal(gh->getLfgProposalId(), false);
        }
        return 0;
    }},

    {"GetLFGQueuedList", [](lua_State* L) -> int {
        auto* gh = getGameHandler(L);
        lua_newtable(L);
        if (gh && gh->isLfgQueued()) {
            const uint32_t id = gh->getLfgDungeonId();
            if (id != 0) {
                lua_pushboolean(L, 1);
                lua_rawseti(L, -2, static_cast<lua_Integer>(id));
            }
        }
        return 1;
    }},
    // Whether a row can be queued for at all. Nothing is locked here - see
    // GetLFDChoiceLockedState - so the answer is yes and the server decides.
    {"IsLFGDungeonJoinable", luaReturnTrue},
    // GetLFGQueueStats() → hasData, leaderNeeds, tankNeeds, healerNeeds,
    //   dpsNeeds, instanceType, instanceName, averageWait, tankWait,
    //   healerWait, damageWait, myWait, queuedTime
    //
    // This answered nil because the numbers never arrived: the queue-status
    // handler required a longer packet than the server sends and returned
    // before reading any of it. With that fixed the whole row is real - how
    // many of each role the queue still wants, and how long each has been
    // waiting.
    {"GetLFGQueueStats", [](lua_State* L) -> int {
        auto* gh = getGameHandler(L);
        if (!gh || !gh->isLfgQueued()) { lua_pushboolean(L, 0); return 1; }
        const uint32_t dungeonId = gh->getLfgDungeonId();
        const game::LfgDungeon* d = nullptr;
        for (const auto& e : gh->getLfgDungeons()) {
            if (e.id == dungeonId) { d = &e; break; }
        }
        // Seconds on the wire; the panel prints them with SecondsToTime.
        auto wait = [](int32_t v) { return v >= 0 ? static_cast<double>(v) : 0.0; };

        lua_pushboolean(L, 1);                                   // 1: hasData
        lua_pushnumber(L, 0);                                    // 2: leaderNeeds
        lua_pushnumber(L, gh->getLfgNeedTank());                 // 3: tankNeeds
        lua_pushnumber(L, gh->getLfgNeedHealer());               // 4: healerNeeds
        lua_pushnumber(L, gh->getLfgNeedDps());                  // 5: dpsNeeds
        lua_pushnumber(L, d ? d->typeId : 1);                    // 6: instanceType
        lua_pushstring(L, d ? d->name.c_str() : "");             // 7: instanceName
        lua_pushnumber(L, gh->getLfgAvgWaitSec());               // 8: averageWait
        lua_pushnumber(L, wait(gh->getLfgWaitTank()));           // 9: tankWait
        lua_pushnumber(L, wait(gh->getLfgWaitHealer()));         // 10: healerWait
        lua_pushnumber(L, wait(gh->getLfgWaitDps()));            // 11: damageWait
        // The player's own estimate, which is the one the panel prints as
        // "Average Wait Time" - it was being given the time already waited, so
        // the line described the past rather than the wait. -1 passes through:
        // the panel hides the line for it, which is what the server not
        // knowing yet should look like.
        lua_pushnumber(L, gh->getLfgMyWaitSec());                // 12: myWait
        // A moment, not a duration: LFDSearchStatus_Update computes
        // GetTime() - queuedTime for the timer it counts up. Handed the
        // duration, it subtracted a minute from the clock and reported a queue
        // entered when the client started.
        lua_pushnumber(L, luaGetTimeNow() -
                          static_cast<double>(gh->getLfgQueuedSeconds())); // 13: queuedTime
        return 13;
    }},

    // ---- Detail this client does not keep ----
    //
    // Each of these is a real feature answered empty, not a feature that does
    // not exist. Named separately so a later pass can see what is left.

    // GetLFGProposalMember(index) → isLeader, role, level, responded,
    //                                accepted, name, class
    //
    // The proposal does carry its group - role, whether it is you, whether the
    // player has answered and what they said - once the header is read at the
    // right offsets. What it does not carry is names, levels or classes: the
    // server sends none, so those stay nil rather than being invented, and the
    // dialog draws a role icon and a tick per row, which is what it is for.
    {"GetLFGProposalMember", [](lua_State* L) -> int {
        auto* gh = getGameHandler(L);
        const int index = static_cast<int>(luaL_optinteger(L, 1, 0));
        if (!gh || index < 1) return luaReturnNil(L);
        const auto& members = gh->getLfgProposalMembers();
        if (index > static_cast<int>(members.size())) return luaReturnNil(L);
        const auto& m = members[static_cast<size_t>(index) - 1];
        lua_pushboolean(L, 0);                       // 1: isLeader - not sent
        // The same tokens GetTexCoordsForRole indexes by. The mask is the one
        // SetLFGRoles sends: 2 tank, 4 healer, 8 damage.
        lua_pushstring(L, (m.role & 0x02) ? "TANK"
                        : (m.role & 0x04) ? "HEALER"
                                          : "DAMAGER");   // 2: role
        lua_pushnil(L);                              // 3: level - not sent
        lua_pushboolean(L, m.answered ? 1 : 0);      // 4: responded
        lua_pushboolean(L, m.accepted ? 1 : 0);      // 5: accepted
        lua_pushnil(L);                              // 6: name - not sent
        lua_pushnil(L);                              // 7: class - not sent
        return 7;
    }},
    {"GetLFGProposalEncounter", [](lua_State* L) -> int { return luaReturnNil(L); }},
    // GetLFGBootProposal() → inProgress, didVote, myVote, targetName,
    //                          totalVotes, bootVotes, timeLeft, reason
    //
    // Answered nil under a note saying the target's name and the reason were
    // not parsed. The reason was parsed all along, and the name is the victim
    // guid the same handler reads - the packet carries no name, which is why
    // looking for one found nothing. The three flags in front of the guid were
    // read and dropped; without them the dialog cannot tell a vote it has
    // already answered from one it has not.
    {"GetLFGBootProposal", [](lua_State* L) -> int {
        auto* gh = getGameHandler(L);
        if (!gh || !gh->isLfgBootInProgress()) return luaReturnNil(L);
        lua_pushboolean(L, 1);
        lua_pushboolean(L, gh->hasLfgBootVoted() ? 1 : 0);
        lua_pushboolean(L, gh->getLfgBootMyVote() ? 1 : 0);
        // Sent as it is even when the guid has not resolved yet. lfdframe gates
        // the popup on this being truthy, and an empty string is truthy in Lua
        // - so a name still in flight costs a blank in the question rather than
        // the vote itself, and the victim is a party member whose name the
        // roster almost always already has.
        lua_pushstring(L, gh->getLfgBootTargetName().c_str());
        lua_pushnumber(L, gh->getLfgBootTotal());
        lua_pushnumber(L, gh->getLfgBootVotes());
        lua_pushnumber(L, gh->getLfgBootTimeLeft());
        lua_pushstring(L, gh->getLfgBootReason().c_str());
        return 8;
    }},
    // GetLFGDungeonRewards(dungeonID) → doneToday, moneyBase, moneyVar,
    //                                    experienceBase, experienceVar, numRewards
    //
    // Numbers, not nil, and that matters more than the values:
    // LFDDungeonReadyDialog_UpdateRewards does
    // `moneyBase + moneyVar * numRandoms` on the line after asking, so a nil
    // raises - and it runs from the same branch of the ready dialog that draws
    // the rest, which is the branch a fresh pop opens in.
    //
    // The server sends one money and one experience figure with the variable
    // parts zeroed, so the totals come out right either way.
    {"GetLFGDungeonRewards", [](lua_State* L) -> int {
        auto* gh = getGameHandler(L);
        const uint32_t dungeonId = static_cast<uint32_t>(luaL_optinteger(L, 1, 0));
        const game::LfgReward* r = nullptr;
        if (gh) {
            for (const auto& e : gh->getLfgRewards()) {
                if (e.dungeonId == dungeonId) { r = &e; break; }
            }
        }
        lua_pushboolean(L, r && r->done ? 1 : 0);                    // doneToday
        lua_pushnumber(L, r ? r->money : 0);                         // moneyBase
        lua_pushnumber(L, 0);                                        // moneyVar
        lua_pushnumber(L, r ? r->experience : 0);                    // experienceBase
        lua_pushnumber(L, 0);                                        // experienceVar
        lua_pushnumber(L, r ? static_cast<double>(r->items.size()) : 0);  // numRewards
        return 6;
    }},
    // GetLFGDungeonRewardInfo(dungeonID, index) → name, texturePath, quantity
    //
    // The caller guards the texture with "we may be waiting on the item data
    // to come from the server", which is exactly this client's position before
    // an item query answers.
    // The four the dungeon finder calls and nothing answered.
    //
    // Two of them run on their own: LFDDungeonReadyPopup asks for both lock
    // infos in its OnShow, and suppressing that popup does not stop its
    // handlers - a hidden frame still runs them - so with LFG_PROPOSAL_SHOW
    // fired, forming a group raised on a nil global whether or not this
    // element had been handed over.
    // Sorting the raid browser's result list by a column. The browser itself is
    // not built here - the SearchLFG family it belongs to is deliberately
    // unfinished - but this one is an OnClick on a column header, so leaving it
    // unbound meant clicking a header raised instead of doing nothing.
    {"SearchLFGSort", [](lua_State* L) -> int { (void)L; return 0; }},
    {"LeaveLFG", [](lua_State* L) -> int {
        if (auto* gh = getGameHandler(L)) gh->lfgLeave();
        return 0;
    }},
    {"RequestLFDPlayerLockInfo", [](lua_State* L) -> int {
        if (auto* gh = getGameHandler(L)) gh->requestLfgPlayerLockInfo();
        return 0;
    }},
    {"RequestLFDPartyLockInfo", [](lua_State* L) -> int {
        if (auto* gh = getGameHandler(L)) gh->requestLfgPartyLockInfo();
        return 0;
    }},
    // GetLFGDungeonRewardLink(dungeonID, index) → the item link for a reward,
    // which shift-clicking a reward icon puts in chat. Same row
    // GetLFGDungeonRewardInfo below reads, said as a link.
    {"GetLFGDungeonRewardLink", [](lua_State* L) -> int {
        auto* gh = getGameHandler(L);
        const uint32_t dungeonId = static_cast<uint32_t>(luaL_optinteger(L, 1, 0));
        const int index = static_cast<int>(luaL_optinteger(L, 2, 0));
        if (!gh || index < 1) return luaReturnNil(L);
        for (const auto& e : gh->getLfgRewards()) {
            if (e.dungeonId != dungeonId) continue;
            if (index > static_cast<int>(e.items.size())) return luaReturnNil(L);
            const auto& item = e.items[static_cast<size_t>(index) - 1];
            gh->ensureItemInfo(item.itemId);
            const auto* info = gh->getItemInfo(item.itemId);
            if (!info || !info->valid) return luaReturnNil(L);
            lua_pushstring(L, game::buildItemLink(item.itemId, info->quality,
                                                  info->name).c_str());
            return 1;
        }
        return luaReturnNil(L);
    }},
    {"GetLFGDungeonRewardInfo", [](lua_State* L) -> int {
        auto* gh = getGameHandler(L);
        const uint32_t dungeonId = static_cast<uint32_t>(luaL_optinteger(L, 1, 0));
        const int index = static_cast<int>(luaL_optinteger(L, 2, 0));
        if (!gh || index < 1) return luaReturnNil(L);
        for (const auto& e : gh->getLfgRewards()) {
            if (e.dungeonId != dungeonId) continue;
            if (index > static_cast<int>(e.items.size())) return luaReturnNil(L);
            const auto& item = e.items[static_cast<size_t>(index) - 1];
            gh->ensureItemInfo(item.itemId);
            const auto* info = gh->getItemInfo(item.itemId);
            lua_pushstring(L, info ? info->name.c_str() : "");
            const std::string icon = gh->getItemIconPath(item.displayInfoId);
            if (icon.empty()) lua_pushnil(L); else lua_pushstring(L, icon.c_str());
            lua_pushnumber(L, item.count);
            return 3;
        }
        return luaReturnNil(L);
    }},
    // The random dungeon the picker opens on. The first one offered that is
    // not locked, which is what "best choice" means with nothing better to
    // rank by; nil rather than zero when there is none, because the caller
    // tests it and falls back to the specific-dungeon list.
    {"GetRandomDungeonBestChoice", [](lua_State* L) -> int {
        auto* gh = getGameHandler(L);
        if (!gh) return luaReturnNil(L);
        const auto& locks = gh->getLfgLocks();
        for (const auto& r : gh->getLfgRewards()) {
            auto it = locks.find(r.dungeonId);
            if (it != locks.end() && it->second != 0) continue;
            lua_pushinteger(L, static_cast<lua_Integer>(r.dungeonId));
            return 1;
        }
        return luaReturnNil(L);
    }},
    // The random dungeon's own row, which the picker shows above the list.
    // GetNumRandomDungeons() - how many of the catalogue are the random kind.
    //
    // Unbound, so opening the dungeon finder's type dropdown called a nil
    // global and raised. Type 6 is random, which is the same field
    // GetLFGRandomDungeonInfo below reports.
    {"GetNumRandomDungeons", [](lua_State* L) -> int {
        auto* gh = getGameHandler(L);
        int n = 0;
        if (gh) {
            for (const auto& d : gh->getLfgDungeons()) {
                if (d.typeId == 6) ++n;
            }
        }
        lua_pushinteger(L, n);
        return 1;
    }},
    // GetLFGRandomDungeonInfo(index) → id, name
    //
    // An index into the random dungeons, counted the same way as the call
    // above, and the id first. This took a dungeon id and answered name and
    // type - so the one caller, which walks 1..GetNumRandomDungeons and reads
    // id and name, looked up dungeon 1, then dungeon 2, and put the name into
    // the id and the type into the name. IsLFGDungeonJoinable was then asked
    // about a string.
    {"GetLFGRandomDungeonInfo", [](lua_State* L) -> int {
        auto* gh = getGameHandler(L);
        const int index = static_cast<int>(luaL_optinteger(L, 1, 0));
        if (!gh || index < 1) return luaReturnNil(L);
        int n = 0;
        for (const auto& d : gh->getLfgDungeons()) {
            if (d.typeId != 6) continue;
            if (++n != index) continue;
            lua_pushinteger(L, static_cast<lua_Integer>(d.id));
            lua_pushstring(L, d.name.c_str());
            return 2;
        }
        return luaReturnNil(L);
    }},
    // GetLFGDungeonInfo(id) - the same twelve values as a row of the info
    // table, asked one at a time. Answered from the same catalogue.
    {"GetLFGDungeonInfo", [](lua_State* L) -> int {
        auto* gh = getGameHandler(L);
        const uint32_t id = static_cast<uint32_t>(luaL_optinteger(L, 1, 0));
        if (!gh || id == 0) return luaReturnNil(L);
        for (const auto& d : gh->getLfgDungeons()) {
            if (d.id != id) continue;
            lua_pushstring(L, d.name.c_str());
            lua_pushinteger(L, static_cast<lua_Integer>(d.typeId));
            lua_pushinteger(L, static_cast<lua_Integer>(d.minLevel));
            lua_pushinteger(L, static_cast<lua_Integer>(d.maxLevel));
            lua_pushinteger(L, static_cast<lua_Integer>(d.recLevel));
            lua_pushinteger(L, static_cast<lua_Integer>(d.minRecLevel));
            lua_pushinteger(L, static_cast<lua_Integer>(d.maxRecLevel));
            lua_pushinteger(L, static_cast<lua_Integer>(d.expansion));
            lua_pushinteger(L, headerIdFor(d.groupId));
            lua_pushstring(L, d.texture.c_str());
            lua_pushinteger(L, static_cast<lua_Integer>(d.difficulty));
            lua_pushinteger(L, maxPlayersFor(d));
            // Fourteen, not twelve. LFG_RETURN_VALUES names the first dozen and
            // stops, but LFDQueueFrameRandom_UpdateFrame reads two more off the
            // end - and it is the last of them that decides whether a dungeon
            // is drawn as a holiday at all, so without it the seasonal bosses
            // were listed as ordinary random dungeons with the wrong artwork.
            lua_pushstring(L, d.description.c_str());   // 13: description
            lua_pushboolean(L, d.isHoliday ? 1 : 0);    // 14: isHoliday
            return 14;
        }
        return luaReturnNil(L);
    }},
    // Per-boss lockouts inside a raid the character is saved to. The saved
    // instance list has no encounter breakdown in it.
    {"GetInstanceLockTimeRemainingEncounter", [](lua_State* L) -> int { return luaReturnNil(L); }},
    // The countdown on the bind-or-leave dialog, read every frame by its
    // OnUpdate. A zero closes the dialog, which is what should happen when
    // there is nothing pending.
    {"GetInstanceLockTimeRemaining", [](lua_State* L) -> int {
        auto* gh = getGameHandler(L);
        float secondsLeft = 0.0f;
        bool previouslySaved = false;
        uint32_t completedMask = 0;
        if (!gh || !gh->getInstanceLockPrompt(secondsLeft, previouslySaved, completedMask)) {
            lua_pushnumber(L, 0);
            lua_pushboolean(L, 0);
            lua_pushinteger(L, 0);
            lua_pushinteger(L, 0);
            return 4;
        }
        // The mask says which encounters are done; its population count is how
        // many, and the total comes from the map's own encounter list.
        uint32_t complete = 0;
        for (uint32_t bit = completedMask; bit; bit &= bit - 1) ++complete;
        const uint32_t total = gh->getDungeonEncounterCount(gh->getCurrentMapId(),
                                                            gh->getInstanceDifficulty());
        lua_pushnumber(L, secondsLeft);
        lua_pushboolean(L, previouslySaved ? 1 : 0);
        lua_pushinteger(L, static_cast<lua_Integer>(total));
        lua_pushinteger(L, static_cast<lua_Integer>(complete));
        return 4;
    }},
    // Accept and be saved to the instance, or decline and go to the graveyard.
    // The server does neither until one of the two arrives.
    {"RespondInstanceLock", [](lua_State* L) -> int {
        if (auto* gh = getGameHandler(L)) gh->respondInstanceLock(lua_toboolean(L, 1) != 0);
        return 0;
    }},
    // Whether the character is sitting out a deserter or random-dungeon
    // cooldown. Both are auras; this client does not check for them, and
    // answering yes would grey out a queue the server would have allowed.
    {"UnitHasLFGDeserter",       luaReturnFalse},
    {"UnitHasLFGRandomCooldown", luaReturnFalse},
    // Filling an empty slot in a party already inside. Needs the party's own
    // queue state, which is not tracked.
    {"CanPartyLFGBackfill",      luaReturnFalse},
    {"GetPartyLFGBackfillInfo",  [](lua_State* L) -> int { return luaReturnNil(L); }},
    {"PartyLFGStartBackfill",    luaReturnNothing},
    // A note shown beside the queue. Nothing carries it.
    {"SetLFGComment",            luaReturnNothing},

    // ---- The raid browser ----
    //
    // A separate system from the queue: it searches for groups already forming
    // rather than building one. None of it is implemented, and the counts
    // answer zero so every loop over results runs zero times.
    {"SearchLFGGetNumResults",     luaReturnZero},
    {"SearchLFGGetResults",        [](lua_State* L) -> int { return luaReturnNil(L); }},
    {"SearchLFGGetPartyResults",   [](lua_State* L) -> int { return luaReturnNil(L); }},
    {"SearchLFGGetEncounterResults", [](lua_State* L) -> int { return luaReturnNil(L); }},
    {"SearchLFGGetJoinedID",       luaReturnZero},
    {"SearchLFGJoin",              luaReturnNothing},
    {"SearchLFGLeave",             luaReturnNothing},
    };

    for (const auto& [name, func] : api) {
        lua_pushcfunction(L, func);
        lua_setglobal(L, name);
    }
}

}  // namespace wowee::addons
