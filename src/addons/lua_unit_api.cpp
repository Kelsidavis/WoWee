// lua_unit_api.cpp - Unit query, stats, party/raid, and player state Lua API bindings.
// Extracted from lua_engine.cpp as part of §5.1 (Tame LuaEngine).
#include "game/shapeshift_forms.hpp"
#include "game/bg_score_defs.hpp"
#include "addons/lua_api_helpers.hpp"

namespace wowee::addons {

static int lua_UnitName(lua_State* L) {
    const char* uid = luaL_optstring(L, 1, "player");
    auto* unit = resolveUnit(L, uid);
    // No game object branch here either, for the reason in UnitExists: a name
    // is the one answer about an object that would have been right, and giving
    // it while UnitExists says no is a unit that half exists.
    if (unit && !unit->getName().empty()) {
        lua_pushstring(L, unit->getName().c_str());
    } else {
        // Fallback: party member name for out-of-range members
        auto* gh = getGameHandler(L);
        std::string uidStr(uid);
        toLowerInPlace(uidStr);
        uint64_t guid = gh ? resolveUnitGuid(gh, uidStr) : 0;
        const auto* pm = findPartyMember(gh, guid);
        if (pm && !pm->name.empty()) {
            lua_pushstring(L, pm->name.c_str());
        } else if (gh && guid != 0) {
            // Try player name cache
            const std::string& cached = gh->lookupName(guid);
            lua_pushstring(L, cached.empty() ? "Unknown" : cached.c_str());
        } else {
            lua_pushstring(L, "Unknown");
        }
    }
    return 1;
}


static int lua_UnitHealth(lua_State* L) {
    const char* uid = luaL_optstring(L, 1, "player");
    auto* unit = resolveUnit(L, uid);
    if (unit) {
        lua_pushnumber(L, unit->getHealth());
    } else {
        // Fallback: party member stats for out-of-range members
        auto* gh = getGameHandler(L);
        std::string uidStr(uid);
        toLowerInPlace(uidStr);
        uint64_t guid = gh ? resolveUnitGuid(gh, uidStr) : 0;
        const auto* pm = findPartyMember(gh, guid);
        lua_pushnumber(L, pm ? pm->curHealth : 0);
    }
    return 1;
}

static int lua_UnitHealthMax(lua_State* L) {
    const char* uid = luaL_optstring(L, 1, "player");
    auto* unit = resolveUnit(L, uid);
    if (unit) {
        lua_pushnumber(L, unit->getMaxHealth());
    } else {
        auto* gh = getGameHandler(L);
        std::string uidStr(uid);
        toLowerInPlace(uidStr);
        uint64_t guid = gh ? resolveUnitGuid(gh, uidStr) : 0;
        const auto* pm = findPartyMember(gh, guid);
        lua_pushnumber(L, pm ? pm->maxHealth : 0);
    }
    return 1;
}

/// UnitPower(unit, powerType) - the second argument names *which* bar.
///
/// Absent, it means the one the unit is using, which is what the main power
/// bar wants. Given, it names one of the seven, and alternatepowerbar.lua is
/// the caller that needs it: a druid in a form has energy or rage as its
/// current power and mana as an alternate, and that frame asks for the
/// alternate by index. Ignoring the argument answered with the current power,
/// so the mana bar shown beside a cat's energy bar was that same energy.
///
/// The seven are all tracked - Entity keeps powers[7] and maxPowers[7] - so
/// this is a matter of reading the one asked for.
static int lua_UnitPower(lua_State* L) {
    const char* uid = luaL_optstring(L, 1, "player");
    const bool byType = lua_isnumber(L, 2);
    const uint8_t want = byType ? static_cast<uint8_t>(lua_tonumber(L, 2)) : 0;
    auto* unit = resolveUnit(L, uid);
    if (unit) {
        lua_pushnumber(L, byType ? unit->getPowerByType(want) : unit->getPower());
    } else {
        auto* gh = getGameHandler(L);
        std::string uidStr(uid);
        toLowerInPlace(uidStr);
        uint64_t guid = gh ? resolveUnitGuid(gh, uidStr) : 0;
        const auto* pm = findPartyMember(gh, guid);
        // Party stats carry one power and say which it is, so a request for a
        // different one is answered with nothing rather than with that.
        const bool same = pm && (!byType || pm->powerType == want);
        lua_pushnumber(L, same ? pm->curPower : 0);
    }
    return 1;
}

/// UnitPowerMax(unit, powerType) - the same, and the one that decides whether
/// the alternate bar is drawn at all: alternatepowerbar tests this against zero
/// before showing itself.
static int lua_UnitPowerMax(lua_State* L) {
    const char* uid = luaL_optstring(L, 1, "player");
    const bool byType = lua_isnumber(L, 2);
    const uint8_t want = byType ? static_cast<uint8_t>(lua_tonumber(L, 2)) : 0;
    auto* unit = resolveUnit(L, uid);
    if (unit) {
        lua_pushnumber(L, byType ? unit->getMaxPowerByType(want) : unit->getMaxPower());
    } else {
        auto* gh = getGameHandler(L);
        std::string uidStr(uid);
        toLowerInPlace(uidStr);
        uint64_t guid = gh ? resolveUnitGuid(gh, uidStr) : 0;
        const auto* pm = findPartyMember(gh, guid);
        const bool same = pm && (!byType || pm->powerType == want);
        lua_pushnumber(L, same ? pm->maxPower : 0);
    }
    return 1;
}

static int lua_UnitLevel(lua_State* L) {
    const char* uid = luaL_optstring(L, 1, "player");
    auto* unit = resolveUnit(L, uid);
    if (unit) {
        lua_pushnumber(L, unit->getLevel());
    } else {
        auto* gh = getGameHandler(L);
        std::string uidStr(uid);
        toLowerInPlace(uidStr);
        uint64_t guid = gh ? resolveUnitGuid(gh, uidStr) : 0;
        const auto* pm = findPartyMember(gh, guid);
        lua_pushnumber(L, pm ? pm->level : 0);
    }
    return 1;
}

static int lua_UnitExists(lua_State* L) {
    const char* uid = luaL_optstring(L, 1, "player");
    auto* unit = resolveUnit(L, uid);
    // A game object is not a unit, and saying otherwise here does not stop at
    // the name. This client targets objects because its right-click path reads
    // the selection back, and its own target frame drew one greyed out and
    // asked nothing further. FrameXML asks a great deal further - dead, level,
    // classification, reaction - and every one of those reads the zero behind
    // an object that has no such field, so a targeted mailbox came up dead, at
    // skull level, with an attackable portrait. Retail never targets one at
    // all; the selection stays for the interaction path, and the interface is
    // told what it asked, which is that there is no unit here.
    if (unit) {
        lua_pushboolean(L, 1);
    } else {
        // Party members in other zones don't have entities but still "exist"
        auto* gh = getGameHandler(L);
        std::string uidStr(uid);
        toLowerInPlace(uidStr);
        uint64_t guid = gh ? resolveUnitGuid(gh, uidStr) : 0;
        lua_pushboolean(L, guid != 0 && findPartyMember(gh, guid) != nullptr);
    }
    return 1;
}

static int lua_UnitIsDead(lua_State* L) {
    const char* uid = luaL_optstring(L, 1, "player");
    auto* unit = resolveUnit(L, uid);
    if (unit) {
        lua_pushboolean(L, unit->getHealth() == 0);
    } else {
        // Fallback: party member stats for out-of-range members
        auto* gh = getGameHandler(L);
        std::string uidStr(uid);
        toLowerInPlace(uidStr);
        uint64_t guid = gh ? resolveUnitGuid(gh, uidStr) : 0;
        const auto* pm = findPartyMember(gh, guid);
        lua_pushboolean(L, pm ? (pm->curHealth == 0 && pm->maxHealth > 0) : 0);
    }
    return 1;
}

static int lua_UnitClass(lua_State* L) {
    const char* uid = luaL_optstring(L, 1, "player");
    auto* gh = getGameHandler(L);
    auto* unit = resolveUnit(L, uid);
    if (unit && gh) {

        uint8_t classId = 0;
        std::string uidStr(uid);
        toLowerInPlace(uidStr);
        if (uidStr == "player") {
            classId = gh->getPlayerClass();
        } else {
            // Read class from UNIT_FIELD_BYTES_0 (class is byte 1)
            uint64_t guid = resolveUnitGuid(gh, uidStr);
            if (guid != 0) {
                auto entity = gh->getEntityManager().getEntity(guid);
                if (entity) {
                    uint32_t bytes0 = entity->getField(
                        game::fieldIndex(game::UF::UNIT_FIELD_BYTES_0));
                    classId = static_cast<uint8_t>((bytes0 >> 8) & 0xFF);
                }
            }
            // Fallback: check name query class/race cache
            if (classId == 0 && guid != 0) {
                classId = gh->lookupPlayerClass(guid);
            }
        }
        // Nothing at all when the class is not known, because that is what WoW
        // answers for a unit it cannot see and what every caller is written
        // against: `local _, class = UnitClass(self.unit); if ( class ) then`.
        //
        // "UNKNOWN" passes that guard and then indexes nothing -
        // CLASS_ICON_TCOORDS has no such key - so the line after it unpacked a
        // nil. ArenaEnemyFrame_OnLoad does exactly this, which took the whole
        // of Blizzard_ArenaUI down at load. A binding that answers a constant
        // where the real one answers nothing defeats every guard written for
        // it, and the guard is the caller saying it already knows how to cope.
        //
        // The token decides: it is the one thing that knows which class ids
        // WoW actually uses, and the two callers in the social API ask it the
        // same question for the same reason.
        const char* token = luaClassToken(classId);
        if (!token) return 0;
        // Second is the uppercase token, not the display name again. Every
        // class-indexed table in FrameXML is keyed by it, so answering "Mage"
        // where "MAGE" was meant looked up nothing: SetPortraitTexture found no
        // CLASS_ICON_TCOORDS entry and fell back to the placeholder portrait,
        // which is the blue question mark on the character sheet.
        lua_pushstring(L, kLuaClasses[classId]);
        lua_pushstring(L, token);
        lua_pushnumber(L, classId);
        return 3;
    }
    // The unit does not exist, which is the case WoW answers nothing for. See
    // above: a constant here reads as a class the caller then looks up.
    return 0;
}

// UnitIsGhost(unit) - true if unit is in ghost form
static int lua_UnitIsGhost(lua_State* L) {
    const char* uid = luaL_optstring(L, 1, "player");
    auto* gh = getGameHandler(L);
    if (!gh) { return luaReturnFalse(L); }
    std::string uidStr(uid);
    toLowerInPlace(uidStr);
    if (uidStr == "player") {
        lua_pushboolean(L, gh->isPlayerGhost());
    } else {
        // Check UNIT_FIELD_FLAGS for UNIT_FLAG_GHOST (0x00000100) - best approximation
        uint64_t guid = resolveUnitGuid(gh, uidStr);
        bool ghost = false;
        if (guid != 0) {
            auto entity = gh->getEntityManager().getEntity(guid);
            if (entity) {
                // Ghost is PLAYER_FLAGS bit 0x10, NOT UNIT_FIELD_FLAGS bit 0x100
                // (which is UNIT_FLAG_IMMUNE_TO_PC - would flag immune NPCs as ghosts).
                uint32_t pf = entity->getField(game::fieldIndex(game::UF::PLAYER_FLAGS));
                ghost = (pf & 0x00000010) != 0;
            }
        }
        lua_pushboolean(L, ghost);
    }
    return 1;
}

// UnitIsDeadOrGhost(unit)
static int lua_UnitIsDeadOrGhost(lua_State* L) {
    const char* uid = luaL_optstring(L, 1, "player");
    auto* unit = resolveUnit(L, uid);
    auto* gh = getGameHandler(L);
    bool dead = (unit && unit->getHealth() == 0);
    if (!dead && gh) {
        std::string uidStr(uid);
        toLowerInPlace(uidStr);
        if (uidStr == "player") dead = gh->isPlayerGhost() || gh->isPlayerDead();
    }
    lua_pushboolean(L, dead);
    return 1;
}

// UnitIsAFK(unit), UnitIsDND(unit)
/// A unit predicate answered from one bit of PLAYER_FLAGS.
///
/// Answers 1 or nil, the way WoW answers a Unit predicate. bnet.lua tests
/// `UnitIsAFK("player") == 1`, which a boolean fails silently, and the two
/// chat sites test it for truth - 1 and nil satisfy both, where true/false
/// and 1/0 each break one of them.
///
/// PLAYER_FLAGS, not UNIT_FIELD_FLAGS: 0x01 there is UNIT_FLAG_SERVER_CONTROLLED
/// and has nothing to do with being away.
static int pushPlayerFlagPredicate(lua_State* L, uint32_t bit) {
    const char* uid = luaL_optstring(L, 1, "player");
    auto* gh = getGameHandler(L);
    if (!gh) { return luaReturnFalse(L); }
    std::string uidStr(uid);
    toLowerInPlace(uidStr);
    uint64_t guid = resolveUnitGuid(gh, uidStr);
    if (guid != 0) {
        auto entity = gh->getEntityManager().getEntity(guid);
        if (entity) {
            uint32_t playerFlags = entity->getField(game::fieldIndex(game::UF::PLAYER_FLAGS));
            if (playerFlags & bit) lua_pushnumber(L, 1); else lua_pushnil(L);
            return 1;
        }
    }
    lua_pushnil(L);
    return 1;
}

static int lua_UnitIsAFK(lua_State* L) {
    return pushPlayerFlagPredicate(L, 0x01);
}

static int lua_UnitIsDND(lua_State* L) {
    return pushPlayerFlagPredicate(L, 0x02);
}

// UnitPlayerControlled(unit) - true for players and player-controlled pets
static int lua_UnitPlayerControlled(lua_State* L) {
    const char* uid = luaL_optstring(L, 1, "player");
    auto* gh = getGameHandler(L);
    if (!gh) { return luaReturnFalse(L); }
    std::string uidStr(uid);
    toLowerInPlace(uidStr);
    uint64_t guid = resolveUnitGuid(gh, uidStr);
    if (guid == 0) { return luaReturnFalse(L); }
    auto entity = gh->getEntityManager().getEntity(guid);
    if (!entity) { return luaReturnFalse(L); }
    // Players are always player-controlled; pets check UNIT_FLAG_PLAYER_CONTROLLED (0x01000000)
    if (entity->getType() == game::ObjectType::PLAYER) {
        lua_pushboolean(L, 1);
    } else {
        uint32_t flags = entity->getField(game::fieldIndex(game::UF::UNIT_FIELD_FLAGS));
        lua_pushboolean(L, (flags & 0x01000000) != 0);
    }
    return 1;
}

// UnitIsTapped(unit) - true if mob is tapped (tagged by any player)
static int lua_UnitIsTapped(lua_State* L) {
    const char* uid = luaL_optstring(L, 1, "target");
    auto* unit = resolveUnit(L, uid);
    if (!unit) { return luaReturnFalse(L); }
    lua_pushboolean(L, (unit->getDynamicFlags() & 0x0004) != 0); // UNIT_DYNFLAG_TAPPED_BY_PLAYER
    return 1;
}

// UnitIsTappedByPlayer(unit) - true if tapped by the local player (can loot)
static int lua_UnitIsTappedByPlayer(lua_State* L) {
    const char* uid = luaL_optstring(L, 1, "target");
    auto* unit = resolveUnit(L, uid);
    if (!unit) { return luaReturnFalse(L); }
    uint32_t df = unit->getDynamicFlags();
    // Tapped by player: has TAPPED flag but also LOOTABLE or TAPPED_BY_ALL
    bool tapped = (df & 0x0004) != 0;
    bool lootable = (df & 0x0001) != 0;
    bool sharedTag = (df & 0x0008) != 0;
    lua_pushboolean(L, tapped && (lootable || sharedTag));
    return 1;
}

// UnitIsTappedByAllThreatList(unit) - true if shared-tag mob
static int lua_UnitIsTappedByAllThreatList(lua_State* L) {
    const char* uid = luaL_optstring(L, 1, "target");
    auto* unit = resolveUnit(L, uid);
    if (!unit) { return luaReturnFalse(L); }
    lua_pushboolean(L, (unit->getDynamicFlags() & 0x0008) != 0);
    return 1;
}

namespace {

/// A unit's standing on a mob's threat list: the status WoW reports, its raw
/// threat, and the leader's.
///
/// One place, because UnitThreatSituation and UnitDetailedThreatSituation must
/// agree - the indicator's colour comes from the first and its number from the
/// second, and two copies of the rule would eventually disagree about which
/// unit is tanking while the number beside it said otherwise.
///
/// Answers false when no list has arrived for this mob, which is not the same
/// as being absent from one: the caller falls back to guessing from whom the
/// mob is attacking, and only a list that exists can say "not on it".
struct ThreatStanding {
    int status = 0;          ///< 0 none, 1 threat, 2 insecurely, 3 securely tanking
    uint32_t threat = 0;
    uint32_t topThreat = 0;
    bool onList = false;
};

bool threatStandingFor(game::GameHandler* gh, uint64_t unitGuid, uint64_t mobGuid,
                       ThreatStanding& out) {
    if (!gh || unitGuid == 0 || mobGuid == 0) return false;
    const auto* list = gh->getThreatList(mobGuid);
    if (!list || list->empty()) return false;
    out.topThreat = (*list)[0].threat;
    for (size_t i = 0; i < list->size(); ++i) {
        if ((*list)[i].victimGuid != unitGuid) continue;
        out.onList = true;
        out.threat = (*list)[i].threat;
        if (i != 0) { out.status = 1; return true; }
        // Securely tanking only when the lead is clear. WoW's own rule is a
        // percentage margin over the runner-up; at the head with someone close
        // behind it is "insecurely", which is what turns the indicator amber
        // before it turns red.
        const uint32_t next = (list->size() > 1) ? (*list)[1].threat : 0;
        out.status = (list->size() > 1 && next * 11 >= out.topThreat * 10) ? 2 : 3;
        return true;
    }
    return true;  // the list exists and this unit is not on it
}

}  // namespace

// UnitThreatSituation(unit, mobUnit) → 0=not tanking, 1=not tanking but threat, 2=insecurely tanking, 3=securely tanking
static int lua_UnitThreatSituation(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { return luaReturnZero(L); }
    const char* uid = luaL_optstring(L, 1, "player");
    const char* mobUid = luaL_optstring(L, 2, nullptr);
    std::string uidStr(uid);
    toLowerInPlace(uidStr);
    uint64_t playerUnitGuid = resolveUnitGuid(gh, uidStr);
    if (playerUnitGuid == 0) { return luaReturnZero(L); }
    // If no mob specified, check general combat threat against current target
    uint64_t mobGuid = 0;
    if (mobUid && *mobUid) {
        std::string mStr(mobUid);
        toLowerInPlace(mStr);
        mobGuid = resolveUnitGuid(gh, mStr);
    }
    // The mob's own threat list first, which is what the server actually sent.
    // This used to go straight to the guess below because the list was built
    // from a misread packet and was never worth consulting - SMSG_THREAT_UPDATE
    // was read as though it carried the second guid that only
    // SMSG_HIGHEST_THREAT_UPDATE has.
    ThreatStanding standing;
    if (threatStandingFor(gh, playerUnitGuid, mobGuid, standing)) {
        lua_pushnumber(L, standing.status);
        return 1;
    }
    // Approximate threat: check if the mob is targeting this unit
    if (mobGuid != 0) {
        auto mobEntity = gh->getEntityManager().getEntity(mobGuid);
        if (mobEntity) {
            if (game::unitTargetGuid(*mobEntity) == playerUnitGuid) {
                lua_pushnumber(L, 3); // securely tanking
                return 1;
            }
        }
    }
    // Check if player is in combat (basic threat indicator)
    if (playerUnitGuid == gh->getPlayerGuid() && gh->isInCombat()) {
        lua_pushnumber(L, 1); // in combat but not tanking
        return 1;
    }
    lua_pushnumber(L, 0);
    return 1;
}

// UnitDetailedThreatSituation(unit, mobUnit) → isTanking, status, threatPct, rawThreatPct, threatValue
static int lua_UnitDetailedThreatSituation(lua_State* L) {
    // Use UnitThreatSituation logic for the basics
    auto* gh = getGameHandler(L);
    if (!gh) {
        lua_pushboolean(L, 0); lua_pushnumber(L, 0); lua_pushnumber(L, 0); lua_pushnumber(L, 0); lua_pushnumber(L, 0);
        return 5;
    }
    const char* uid = luaL_optstring(L, 1, "player");
    const char* mobUid = luaL_optstring(L, 2, nullptr);
    std::string uidStr(uid);
    toLowerInPlace(uidStr);
    uint64_t unitGuid = resolveUnitGuid(gh, uidStr);
    uint64_t mobGuid = 0;
    if (mobUid && *mobUid) {
        std::string mStr(mobUid);
        toLowerInPlace(mStr);
        mobGuid = resolveUnitGuid(gh, mStr);
    }

    ThreatStanding standing;
    if (threatStandingFor(gh, unitGuid, mobGuid, standing)) {
        const bool isTanking = (standing.status >= 2);
        double rawPct = 0.0;
        if (standing.topThreat > 0)
            rawPct = 100.0 * standing.threat / standing.topThreat;
        // Against the point where aggro changes hands rather than against the
        // leader: WoW pulls at a tenth clear of the current tank, so a hundred
        // here means "about to take it" and is what the numeric indicator
        // colours on. Whoever holds it is already at a hundred by definition.
        const double pullPct = isTanking ? 100.0 : std::min(100.0, rawPct / 1.1);
        lua_pushboolean(L, isTanking);
        lua_pushnumber(L, standing.status);
        lua_pushnumber(L, pullPct);
        lua_pushnumber(L, rawPct);
        lua_pushnumber(L, standing.threat);
        return 5;
    }

    // No list for this mob yet. The same guess UnitThreatSituation falls back
    // to, and the same lack of a number behind it.
    bool isTanking = false;
    int status = 0;
    if (unitGuid != 0 && mobGuid != 0) {
        auto mobEnt = gh->getEntityManager().getEntity(mobGuid);
        if (mobEnt && game::unitTargetGuid(*mobEnt) == unitGuid) {
            isTanking = true;
            status = 3;
        }
    }
    lua_pushboolean(L, isTanking);
    lua_pushnumber(L, status);
    lua_pushnumber(L, isTanking ? 100.0 : 0.0); // threatPct
    lua_pushnumber(L, isTanking ? 100.0 : 0.0); // rawThreatPct
    lua_pushnumber(L, 0); // threatValue - unknown without the list
    return 5;
}

// UnitDistanceSquared(unit) → distSq, canCalculate
static int lua_UnitDistanceSquared(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { lua_pushnumber(L, 0); lua_pushboolean(L, 0); return 2; }
    const char* uid = luaL_checkstring(L, 1);
    std::string uidStr(uid);
    toLowerInPlace(uidStr);
    uint64_t guid = resolveUnitGuid(gh, uidStr);
    if (guid == 0 || guid == gh->getPlayerGuid()) { lua_pushnumber(L, 0); lua_pushboolean(L, 0); return 2; }
    auto targetEnt = gh->getEntityManager().getEntity(guid);
    auto playerEnt = gh->getEntityManager().getEntity(gh->getPlayerGuid());
    if (!targetEnt || !playerEnt) { lua_pushnumber(L, 0); lua_pushboolean(L, 0); return 2; }
    float dx = playerEnt->getX() - targetEnt->getX();
    float dy = playerEnt->getY() - targetEnt->getY();
    float dz = playerEnt->getZ() - targetEnt->getZ();
    lua_pushnumber(L, dx*dx + dy*dy + dz*dz);
    lua_pushboolean(L, 1);
    return 2;
}

// CheckInteractDistance(unit, distIndex) → boolean
// distIndex: 1=inspect(28yd), 2=trade(11yd), 3=duel(10yd), 4=follow(28yd)
static int lua_CheckInteractDistance(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { return luaReturnFalse(L); }
    const char* uid = luaL_checkstring(L, 1);
    int distIdx = static_cast<int>(luaL_optnumber(L, 2, 4));
    std::string uidStr(uid);
    toLowerInPlace(uidStr);
    uint64_t guid = resolveUnitGuid(gh, uidStr);
    if (guid == 0) { return luaReturnFalse(L); }
    auto targetEnt = gh->getEntityManager().getEntity(guid);
    auto playerEnt = gh->getEntityManager().getEntity(gh->getPlayerGuid());
    if (!targetEnt || !playerEnt) { return luaReturnFalse(L); }
    float dx = playerEnt->getX() - targetEnt->getX();
    float dy = playerEnt->getY() - targetEnt->getY();
    float dz = playerEnt->getZ() - targetEnt->getZ();
    float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
    float maxDist = 28.0f; // default: follow/inspect range
    switch (distIdx) {
        case 1: maxDist = 28.0f; break; // inspect
        case 2: maxDist = 11.11f; break; // trade
        case 3: maxDist = 9.9f; break; // duel
        case 4: maxDist = 28.0f; break; // follow
    }
    lua_pushboolean(L, dist <= maxDist);
    return 1;
}

// UnitInRange(unit) → inRange, checkedOk
//
// The raid frames dim a member who has gone too far to help. Forty yards is
// what WoW means by it - the range most party-useful spells share - and the
// second return says whether the question could be answered at all: a member
// on another part of the map has no entity here, and dimming them for being
// out of range would be a guess dressed as a measurement.
static int lua_UnitInRange(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { lua_pushboolean(L, 0); lua_pushboolean(L, 0); return 2; }
    std::string uidStr(luaL_optstring(L, 1, "player"));
    toLowerInPlace(uidStr);
    const uint64_t guid = resolveUnitGuid(gh, uidStr);
    auto other  = guid ? gh->getEntityManager().getEntity(guid) : nullptr;
    auto player = gh->getEntityManager().getEntity(gh->getPlayerGuid());
    if (!other || !player) {
        lua_pushboolean(L, 0);
        lua_pushboolean(L, 0);   // could not be checked
        return 2;
    }
    const float dx = player->getX() - other->getX();
    const float dy = player->getY() - other->getY();
    const float dz = player->getZ() - other->getZ();
    lua_pushboolean(L, (dx*dx + dy*dy + dz*dz) <= (40.0f * 40.0f));
    lua_pushboolean(L, 1);
    return 2;
}

// IsRaidLeader() / IsPartyLeader-style check against the group's leader guid.
static int lua_IsRaidLeader(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { return luaReturnFalse(L); }
    const auto& group = gh->getPartyData();
    lua_pushboolean(L, group.leaderGuid != 0 &&
                       group.leaderGuid == gh->getPlayerGuid() ? 1 : 0);
    return 1;
}

// UnitIsVisible(unit) → boolean (entity exists in the client's entity manager)
static int lua_UnitIsVisible(lua_State* L) {
    const char* uid = luaL_optstring(L, 1, "target");
    auto* unit = resolveUnit(L, uid);
    lua_pushboolean(L, unit != nullptr);
    return 1;
}

/// UnitGroupRolesAssigned(unit) → isTank, isHealer, isDamage.
///
/// Three booleans, not a role name. Both callers in this interface are written
/// the same way:
///
///     local isTank, isHealer, isDamage = UnitGroupRolesAssigned(unit)
///     if ( isTank ) then ... elseif ( isHealer ) then ...
///
/// A single string filled the first of those and left the other two nil - and
/// every string is truthy in Lua, "NONE" included, so the first branch always
/// won. The player frame and every party member frame showed the tank icon
/// regardless of role, including for a player with no role and no party.
///
/// Nothing here reads the returned name, so there is no caller to keep happy:
/// playerframe.lua and partymemberframe.lua are the only two.
static int lua_UnitGroupRolesAssigned(lua_State* L) {
    auto pushRoles = [L](bool tank, bool healer, bool damage) {
        lua_pushboolean(L, tank   ? 1 : 0);
        lua_pushboolean(L, healer ? 1 : 0);
        lua_pushboolean(L, damage ? 1 : 0);
        return 3;
    };
    auto* gh = getGameHandler(L);
    if (!gh) return pushRoles(false, false, false);
    const char* uid = luaL_optstring(L, 1, "player");
    std::string uidStr(uid);
    toLowerInPlace(uidStr);
    uint64_t guid = resolveUnitGuid(gh, uidStr);
    if (guid == 0) return pushRoles(false, false, false);
    const auto& pd = gh->getPartyData();
    for (const auto& m : pd.members) {
        if (m.guid != guid) continue;
        // WotLK LFG roles bitmask (from SMSG_GROUP_LIST / SMSG_LFG_ROLE_CHECK_UPDATE).
        // Bit 0x01 = Leader (not a combat role), 0x02 = Tank, 0x04 = Healer, 0x08 = DPS.
        constexpr uint8_t kRoleTank    = 0x02;
        constexpr uint8_t kRoleHealer  = 0x04;
        constexpr uint8_t kRoleDamager = 0x08;
        return pushRoles((m.roles & kRoleTank)   != 0,
                         (m.roles & kRoleHealer) != 0,
                         (m.roles & kRoleDamager) != 0);
    }
    // Not in the party, or in no party at all: no role, and the icon is hidden
    // rather than guessed at.
    return pushRoles(false, false, false);
}

// UnitCanAttack(unit, otherUnit) → boolean
static int lua_UnitCanAttack(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { return luaReturnFalse(L); }
    const char* uid1 = luaL_checkstring(L, 1);
    const char* uid2 = luaL_checkstring(L, 2);
    std::string u1(uid1), u2(uid2);
    toLowerInPlace(u1);
    toLowerInPlace(u2);
    uint64_t g1 = resolveUnitGuid(gh, u1);
    uint64_t g2 = resolveUnitGuid(gh, u2);
    if (g1 == 0 || g2 == 0 || g1 == g2) { return luaReturnFalse(L); }
    // Check if unit2 is hostile to unit1
    auto* unit2 = resolveUnit(L, uid2);
    if (unit2 && unit2->isHostile()) {
        lua_pushboolean(L, 1);
    } else {
        lua_pushboolean(L, 0);
    }
    return 1;
}

// UnitCanCooperate(unit, otherUnit) → boolean
static int lua_UnitCanCooperate(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { return luaReturnFalse(L); }
    (void)luaL_checkstring(L, 1); // unit1 (unused - cooperation is based on unit2's hostility)
    const char* uid2 = luaL_checkstring(L, 2);
    auto* unit2 = resolveUnit(L, uid2);
    if (!unit2) { return luaReturnFalse(L); }
    lua_pushboolean(L, !unit2->isHostile());
    return 1;
}

// UnitCreatureFamily(unit) → familyName or nil
static int lua_UnitCreatureFamily(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { return luaReturnNil(L); }
    const char* uid = luaL_optstring(L, 1, "target");
    std::string uidStr(uid);
    toLowerInPlace(uidStr);
    uint64_t guid = resolveUnitGuid(gh, uidStr);
    if (guid == 0) { return luaReturnNil(L); }
    auto entity = gh->getEntityManager().getEntity(guid);
    if (!entity || entity->getType() == game::ObjectType::PLAYER) { return luaReturnNil(L); }
    auto unit = std::dynamic_pointer_cast<game::Unit>(entity);
    if (!unit) { return luaReturnNil(L); }
    uint32_t family = gh->getCreatureFamily(unit->getEntry());
    if (family == 0) { return luaReturnNil(L); }
    static constexpr const char* kFamilies[] = {
        "", "Wolf", "Cat", "Spider", "Bear", "Boar", "Crocolisk", "Carrion Bird",
        "Crab", "Gorilla", "Raptor", "", "Tallstrider", "", "", "Felhunter",
        "Voidwalker", "Succubus", "", "Doomguard", "Scorpid", "Turtle", "",
        "Imp", "Bat", "Hyena", "Bird of Prey", "Wind Serpent", "", "Dragonhawk",
        "Ravager", "Warp Stalker", "Sporebat", "Nether Ray", "Serpent", "Moth",
        "Chimaera", "Devilsaur", "Ghoul", "Silithid", "Worm", "Rhino", "Wasp",
        "Core Hound", "Spirit Beast"
    };
    lua_pushstring(L, (family < sizeof(kFamilies)/sizeof(kFamilies[0]) && kFamilies[family][0])
        ? kFamilies[family] : "Beast");
    return 1;
}

// UnitOnTaxi(unit) → boolean (true if on a flight path)
static int lua_UnitOnTaxi(lua_State* L) {
    const char* uid = luaL_optstring(L, 1, "player");
    auto* gh = getGameHandler(L);
    if (!gh) { return luaReturnFalse(L); }
    std::string uidStr(uid);
    toLowerInPlace(uidStr);
    if (uidStr == "player") {
        lua_pushboolean(L, gh->isOnTaxiFlight());
    } else {
        lua_pushboolean(L, 0); // Can't determine for other units
    }
    return 1;
}

// UnitSex(unit) → 1=unknown, 2=male, 3=female
static int lua_UnitSex(lua_State* L) {
    const char* uid = luaL_optstring(L, 1, "player");
    auto* gh = getGameHandler(L);
    if (!gh) { lua_pushnumber(L, 1); return 1; }
    std::string uidStr(uid);
    toLowerInPlace(uidStr);
    uint64_t guid = resolveUnitGuid(gh, uidStr);
    if (guid != 0) {
        auto entity = gh->getEntityManager().getEntity(guid);
        if (entity) {
            // Gender is byte 2 of UNIT_FIELD_BYTES_0 (0=male, 1=female)
            uint32_t bytes0 = entity->getField(game::fieldIndex(game::UF::UNIT_FIELD_BYTES_0));
            uint8_t gender = static_cast<uint8_t>((bytes0 >> 16) & 0xFF);
            lua_pushnumber(L, gender == 0 ? 2 : (gender == 1 ? 3 : 1)); // WoW: 2=male, 3=female
            return 1;
        }
    }
    lua_pushnumber(L, 1); // unknown
    return 1;
}

// --- Player/Game API ---

static int lua_UnitStat(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { lua_pushnumber(L, 0); lua_pushnumber(L, 0); lua_pushnumber(L, 0); lua_pushnumber(L, 0); return 4; }
    int statIdx = static_cast<int>(luaL_checknumber(L, 2)) - 1; // WoW API is 1-indexed
    // Whose stat. This ignored the unit and answered from the player, so the
    // paperdoll's pet tab listed the hunter's own Strength as the pet's.
    std::string who(luaL_optstring(L, 1, "player"));
    toLowerInPlace(who);
    int32_t val = 0;
    if (who == "pet") {
        if (statIdx >= 0 && statIdx < 5) val = gh->getPetStats()[static_cast<size_t>(statIdx)];
    } else if (who == "player") {
        val = gh->getPlayerStat(statIdx);
    }
    if (val < 0) val = 0;
    // We only have the effective value from the server; report base=effective, no buffs
    lua_pushnumber(L, val); // base (approximate - server only sends effective)
    lua_pushnumber(L, val); // effective
    lua_pushnumber(L, 0);   // positive buff
    lua_pushnumber(L, 0);   // negative buff
    return 4;
}

// GetDodgeChance() → percent
static int lua_GetDodgeChance(lua_State* L) {
    auto* gh = getGameHandler(L);
    float v = gh ? gh->getDodgePct() : 0.0f;
    lua_pushnumber(L, v >= 0 ? v : 0.0);
    return 1;
}

// GetParryChance() → percent
static int lua_GetParryChance(lua_State* L) {
    auto* gh = getGameHandler(L);
    float v = gh ? gh->getParryPct() : 0.0f;
    lua_pushnumber(L, v >= 0 ? v : 0.0);
    return 1;
}

// GetBlockChance() → percent
static int lua_GetBlockChance(lua_State* L) {
    auto* gh = getGameHandler(L);
    float v = gh ? gh->getBlockPct() : 0.0f;
    lua_pushnumber(L, v >= 0 ? v : 0.0);
    return 1;
}

// GetCritChance() → percent (melee crit)
static int lua_GetCritChance(lua_State* L) {
    auto* gh = getGameHandler(L);
    float v = gh ? gh->getCritPct() : 0.0f;
    lua_pushnumber(L, v >= 0 ? v : 0.0);
    return 1;
}

// GetRangedCritChance() → percent
static int lua_GetRangedCritChance(lua_State* L) {
    auto* gh = getGameHandler(L);
    float v = gh ? gh->getRangedCritPct() : 0.0f;
    lua_pushnumber(L, v >= 0 ? v : 0.0);
    return 1;
}

// GetSpellCritChance(school) → percent  (1=Holy,2=Fire,3=Nature,4=Frost,5=Shadow,6=Arcane)
static int lua_GetSpellCritChance(lua_State* L) {
    auto* gh = getGameHandler(L);
    int school = static_cast<int>(luaL_checknumber(L, 1));
    float v = gh ? gh->getSpellCritPct(school) : 0.0f;
    lua_pushnumber(L, v >= 0 ? v : 0.0);
    return 1;
}

// GetCombatRating(ratingIndex) → value.
// The FrameXML CR_* constants are 1-based (CR_WEAPON_SKILL=1 … CR_ARMOR_PENETRATION=25),
// but the player-field array and the combat game tables are 0-based (AzerothCore's
// CombatRating enum, weapon skill at 0). Convert at the boundary or every rating reads
// its neighbour's slot.
static int lua_GetCombatRating(lua_State* L) {
    auto* gh = getGameHandler(L);
    int cr = static_cast<int>(luaL_checknumber(L, 1)) - 1;
    int32_t v = gh ? gh->getCombatRating(cr) : 0;
    lua_pushnumber(L, v >= 0 ? v : 0);
    return 1;
}

// GetSpellBonusDamage(school) → value  (1-6 magic schools)
static int lua_GetSpellBonusDamage(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { return luaReturnZero(L); }
    int32_t sp = gh->getSpellPower();
    lua_pushnumber(L, sp >= 0 ? sp : 0);
    return 1;
}

// GetSpellBonusHealing() → value
static int lua_GetSpellBonusHealing(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { return luaReturnZero(L); }
    int32_t v = gh->getHealingPower();
    lua_pushnumber(L, v >= 0 ? v : 0);
    return 1;
}

// GetMeleeHaste / GetAttackPowerForStat stubs for addon compat
static int lua_GetAttackPower(lua_State* L) {
    auto* gh = getGameHandler(L);
    int32_t ap = gh ? gh->getMeleeAttackPower() : 0;
    if (ap < 0) ap = 0;
    lua_pushnumber(L, ap);  // base
    lua_pushnumber(L, 0);   // posBuff
    lua_pushnumber(L, 0);   // negBuff
    return 3;
}

static int lua_GetRangedAttackPower(lua_State* L) {
    auto* gh = getGameHandler(L);
    int32_t ap = gh ? gh->getRangedAttackPower() : 0;
    if (ap < 0) ap = 0;
    lua_pushnumber(L, ap);
    lua_pushnumber(L, 0);
    lua_pushnumber(L, 0);
    return 3;
}


static int lua_IsInGroup(lua_State* L) {
    auto* gh = getGameHandler(L);
    lua_pushboolean(L, gh && gh->isInGroup());
    return 1;
}

static int lua_IsInRaid(lua_State* L) {
    auto* gh = getGameHandler(L);
    lua_pushboolean(L, gh && gh->isInGroup() && gh->getPartyData().groupType == 1);
    return 1;
}

// PlaySound(soundId) - play a WoW UI sound by ID or name

static int lua_UnitRace(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { lua_pushstring(L, "Unknown"); lua_pushstring(L, "Unknown"); lua_pushnumber(L, 0); return 3; }
    std::string uid(luaL_optstring(L, 1, "player"));
    toLowerInPlace(uid);

    const uint8_t raceId = unitRaceOf(gh, uid);
    const bool known = (raceId > 0 && raceId < 12);
    const char* name = known ? kLuaRaces[raceId] : "Unknown";
    const char* file = known ? kLuaRaceFileNames[raceId] : "Unknown";
    lua_pushstring(L, name);      // 1: localized race
    lua_pushstring(L, file);      // 2: race file name, spliced into asset paths
    lua_pushnumber(L, raceId);    // 3: raceId (WoW returns 3 values)
    return 3;
}

static int lua_UnitPowerType(lua_State* L) {
    const char* uid = luaL_optstring(L, 1, "player");
    auto* unit = resolveUnit(L, uid);

    if (unit) {
        uint8_t pt = unit->getPowerType();
        lua_pushnumber(L, pt);
        lua_pushstring(L, (pt < 7) ? kLuaPowerNames[pt] : "MANA");
        return 2;
    }
    // Fallback: party member stats for out-of-range members
    auto* gh = getGameHandler(L);
    std::string uidStr(uid);
    toLowerInPlace(uidStr);
    uint64_t guid = gh ? resolveUnitGuid(gh, uidStr) : 0;
    const auto* pm = findPartyMember(gh, guid);
    if (pm) {
        uint8_t pt = pm->powerType;
        lua_pushnumber(L, pt);
        lua_pushstring(L, (pt < 7) ? kLuaPowerNames[pt] : "MANA");
        return 2;
    }
    lua_pushnumber(L, 0);
    lua_pushstring(L, "MANA");
    return 2;
}

static int lua_GetNumGroupMembers(lua_State* L) {
    auto* gh = getGameHandler(L);
    lua_pushnumber(L, gh ? gh->getPartyData().memberCount : 0);
    return 1;
}

static int lua_UnitGUID(lua_State* L) {
    const char* uid = luaL_optstring(L, 1, "player");
    auto* gh = getGameHandler(L);
    if (!gh) { return luaReturnNil(L); }
    std::string uidStr(uid);
    toLowerInPlace(uidStr);
    uint64_t guid = resolveUnitGuid(gh, uidStr);
    if (guid == 0) { return luaReturnNil(L); }
    char buf[32];
    snprintf(buf, sizeof(buf), "0x%016llX", (unsigned long long)guid);
    lua_pushstring(L, buf);
    return 1;
}

/// UnitPVPName(unit) → the name with the player's title around it, or just
/// the name when there is no title.
///
/// The comment here used to say titles were not tracked. They are - the known
/// bits and the chosen one both come off the player - so the character sheet
/// showed a bare name for someone wearing a title.
///
/// Only for the player: a title is a bit index on the unit's own fields and
/// this client reads that for itself alone, so anyone else answers their name,
/// which is what WoW answers for a character wearing none.
static int lua_UnitPVPName(lua_State* L) {
    auto* gh = getGameHandler(L);
    const char* uid = luaL_optstring(L, 1, "player");
    if (gh && uid) {
        std::string id(uid);
        toLowerInPlace(id);
        if (id == "player" && gh->getChosenTitleBit() > 0) {
            const std::string titled =
                gh->getFormattedTitle(static_cast<uint32_t>(gh->getChosenTitleBit()));
            if (!titled.empty()) { lua_pushstring(L, titled.c_str()); return 1; }
        }
    }
    return lua_UnitName(L);
}

static int lua_UnitIsPlayer(lua_State* L) {
    const char* uid = luaL_optstring(L, 1, "player");
    auto* gh = getGameHandler(L);
    if (!gh) { return luaReturnFalse(L); }
    std::string uidStr(uid);
    toLowerInPlace(uidStr);
    uint64_t guid = resolveUnitGuid(gh, uidStr);
    auto entity = guid ? gh->getEntityManager().getEntity(guid) : nullptr;
    lua_pushboolean(L, entity && entity->getType() == game::ObjectType::PLAYER);
    return 1;
}

// Vehicles are not modelled at all, and the honest answer to every question
// about one is no. It matters that these are real rather than left to the
// fallback: PlayerFrame_UpdateStatus asks first thing, and a wrong yes swaps
// the whole frame to vehicle art and hides the rest of it.
static int lua_UnitHasVehicleUI(lua_State* L) {
    (void)L;
    return luaReturnFalse(L);
}
/// Whether that unit is riding a vehicle. Only the player's own state is
/// known - no per-unit vehicle field is parsed - so everyone else answers
/// false, which is what this answered for the player too.
///
/// UnitHasVehicleUI beside it stays false deliberately. Turning it on hands
/// the action bar to VehicleMenuBar, whose own readers - GetVehicleUIIndicator,
/// UnitVehicleSeatInfo, CanEjectPassengerFromSeat - are not bound, so the bar
/// would raise where it now simply does not appear.
static int lua_UnitInVehicle(lua_State* L) {
    auto* gh = getGameHandler(L);
    const char* unit = lua_tostring(L, 1);
    const bool isPlayer = unit && (std::strcmp(unit, "player") == 0);
    lua_pushboolean(L, (isPlayer && gh && gh->isInVehicle()) ? 1 : 0);
    return 1;
}
static int lua_UnitControllingVehicle(lua_State* L) { return luaReturnFalse(L); }
/// The guid held in a two-word unit field, or zero.
///
/// Absent on an expansion whose table does not name the field, which is every
/// one but WotLK for these two - an index that is not known is not guessed at.
static uint64_t guidField(game::GameHandler* gh, uint64_t unitGuid, game::UF field) {
    if (!gh) return 0;
    const uint16_t idx = game::fieldIndex(field);
    if (idx == 0xFFFF) return 0;
    auto entity = gh->getEntityManager().getEntity(unitGuid);
    if (!entity) return 0;
    const uint64_t lo = entity->getField(idx);
    const uint64_t hi = entity->getField(static_cast<uint16_t>(idx + 1));
    return (hi << 32) | lo;
}

/// Whether this unit is driving something else.
///
/// UNIT_FIELD_CHARM names what it controls, and it was not read because the
/// field was not in the table. Both these answered a flat false and said so.
static int lua_UnitIsPossessed(lua_State* L) {
    auto* gh = getGameHandler(L);
    const char* uid = luaL_optstring(L, 1, "player");
    const uint64_t guid = gh ? resolveUnitGuid(gh, uid) : 0;
    lua_pushboolean(L, guidField(gh, guid, game::UF::UNIT_FIELD_CHARM) != 0);
    return 1;
}

/// Whether something else is driving this unit - the other end of the same
/// relationship, in UNIT_FIELD_CHARMEDBY.
///
/// Bound rather than left out because ToggleGameMenu reads it, and it used to
/// survive there only by a short circuit: `ClearTarget() and (not
/// UnitIsCharmed(...))` never reaches the second half because ClearTarget
/// returns nothing. That is a fact about a neighbouring binding's return
/// count, not about this one, and pressing Escape is not where anyone wants to
/// find that out.
static int lua_UnitIsCharmed(lua_State* L) {
    auto* gh = getGameHandler(L);
    const char* uid = luaL_optstring(L, 1, "player");
    const uint64_t guid = gh ? resolveUnitGuid(gh, uid) : 0;
    lua_pushboolean(L, guidField(gh, guid, game::UF::UNIT_FIELD_CHARMEDBY) != 0);
    return 1;
}
static int lua_UnitIsTalking(lua_State* L) { return luaReturnFalse(L); }
static int lua_UnitInBattleground(lua_State* L) { return luaReturnNil(L); }

/// True for a unit in the player's own group, and for its pet. The party
/// frames use this to decide whether a unit is one of ours at all.
static int lua_UnitPlayerOrPetInParty(lua_State* L) {
    const char* uid = luaL_optstring(L, 1, "player");
    auto* gh = getGameHandler(L);
    if (!gh) { return luaReturnFalse(L); }
    std::string uidStr(uid);
    toLowerInPlace(uidStr);
    // "partypet2" asks about party member 2's pet, and the answer is whatever
    // holds for the member.
    const size_t petPos = uidStr.find("pet");
    if (petPos != std::string::npos) uidStr.erase(petPos, 3);
    if (uidStr.empty() || uidStr == "player") {
        lua_pushboolean(L, gh->isInGroup());
        return 1;
    }
    uint64_t guid = resolveUnitGuid(gh, uidStr);
    if (guid == 0) { return luaReturnFalse(L); }
    const auto& pd = gh->getPartyData();
    bool found = false;
    for (const auto& m : pd.members) {
        if (m.guid == guid) { found = true; break; }
    }
    lua_pushboolean(L, found);
    return 1;
}

static int lua_UnitPlayerOrPetInRaid(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh || gh->getPartyData().groupType != 1) { return luaReturnFalse(L); }
    return lua_UnitPlayerOrPetInParty(L);
}

/// Talent points, and the two the interface actually reads: unspent points and
/// the total spent. The micro button flashes on the first.
// UnitCharacterPoints(unit) → talent points, primary professions still learnable
//
// Two values. The trainer's confirmation reads the second bare -
// `if ( cp2 < MAX_LEARNABLE_PROFESSIONS )` - to decide whether it is offering a
// first profession or a second, so returning one value made confirming a
// profession an error rather than a question.
//
// Two is the limit, and a primary profession is category 11 in SkillLine.dbc,
// which is the same test isProfessionSpell uses rather than a second opinion
// about what counts.
static int lua_UnitCharacterPoints(lua_State* L) {
    const char* uid = luaL_optstring(L, 1, "player");
    auto* gh = getGameHandler(L);
    std::string uidStr(uid);
    toLowerInPlace(uidStr);
    if (!gh || uidStr != "player") { lua_pushinteger(L, 0); lua_pushinteger(L, 0); return 2; }

    constexpr int kMaxPrimaryProfessions = 2;
    int known = 0;
    for (const auto& [skillId, skill] : gh->getPlayerSkills()) {
        (void)skill;
        if (gh->getSkillCategory(skillId) == 11) ++known;
    }
    lua_pushinteger(L, static_cast<lua_Integer>(gh->getUnspentTalentPoints()));
    lua_pushinteger(L, std::max(0, kMaxPrimaryProfessions - known));
    return 2;
}

static int lua_PetHasActionBar(lua_State* L) {
    auto* gh = getGameHandler(L);
    lua_pushboolean(L, gh && gh->hasPet());
    return 1;
}
static int lua_PetCanBeDismissed(lua_State* L) {
    auto* gh = getGameHandler(L);
    lua_pushboolean(L, gh && gh->hasPet());
    return 1;
}
// Abandoning is a hunter's pet, and nothing here knows which kind it has yet.
static int lua_PetCanBeAbandoned(lua_State* L) { return luaReturnFalse(L); }

static int lua_InCombatLockdown(lua_State* L) {
    auto* gh = getGameHandler(L);
    lua_pushboolean(L, gh && gh->isInCombat());
    return 1;
}

static int lua_IsMounted(lua_State* L) {
    auto* gh = getGameHandler(L);
    lua_pushboolean(L, gh && gh->isMounted());
    return 1;
}

static int lua_IsFlying(lua_State* L) {
    auto* gh = getGameHandler(L);
    lua_pushboolean(L, gh && gh->isPlayerFlying());
    return 1;
}

static int lua_IsSwimming(lua_State* L) {
    auto* gh = getGameHandler(L);
    lua_pushboolean(L, gh && gh->isSwimming());
    return 1;
}

static int lua_IsResting(lua_State* L) {
    auto* gh = getGameHandler(L);
    lua_pushboolean(L, gh && gh->isPlayerResting());
    return 1;
}

static int lua_IsFalling(lua_State* L) {
    auto* gh = getGameHandler(L);
    // Check FALLING movement flag
    if (!gh) { return luaReturnFalse(L); }
    const auto& mi = gh->getMovementInfo();
    lua_pushboolean(L, (mi.flags & 0x2000) != 0); // MOVEFLAG_FALLING = 0x2000
    return 1;
}

static int lua_IsStealthed(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { return luaReturnFalse(L); }
    // Check for stealth auras (aura flags bit 0x40 = is harmful, stealth is a buff)
    // WoW detects stealth via unit flags: UNIT_FLAG_IMMUNE (0x02) or specific aura IDs
    // Simplified: check player auras for known stealth spell IDs
    bool stealthed = false;
    for (const auto& a : gh->getPlayerAuras()) {
        if (a.isEmpty() || a.spellId == 0) continue;
        // Common stealth IDs: 1784 (Stealth), 5215 (Prowl), 66 (Invisibility)
        if (a.spellId == 1784 || a.spellId == 5215 || a.spellId == 66 ||
            a.spellId == 1785 || a.spellId == 1786 || a.spellId == 1787 ||
            a.spellId == 11305 || a.spellId == 11306) {
            stealthed = true;
            break;
        }
    }
    lua_pushboolean(L, stealthed);
    return 1;
}

static int lua_GetUnitSpeed(lua_State* L) {
    auto* gh = getGameHandler(L);
    const char* uid = luaL_optstring(L, 1, "player");
    if (!gh || std::string(uid) != "player") {
        lua_pushnumber(L, 0);
        return 1;
    }
    lua_pushnumber(L, gh->getServerRunSpeed());
    return 1;
}

// --- Container/Bag API ---
// WoW bags: container 0 = backpack (16 slots), containers 1-4 = equipped bags


static int lua_UnitXP(lua_State* L) {
    const char* uid = luaL_optstring(L, 1, "player");
    auto* gh = getGameHandler(L);
    if (!gh) { return luaReturnZero(L); }
    std::string u(uid);
    toLowerInPlace(u);
    if (u == "player") lua_pushnumber(L, gh->getPlayerXp());
    else lua_pushnumber(L, 0);
    return 1;
}

static int lua_UnitXPMax(lua_State* L) {
    const char* uid = luaL_optstring(L, 1, "player");
    auto* gh = getGameHandler(L);
    if (!gh) { lua_pushnumber(L, 1); return 1; }
    std::string u(uid);
    toLowerInPlace(u);
    if (u == "player") {
        uint32_t nlxp = gh->getPlayerNextLevelXp();
        lua_pushnumber(L, nlxp > 0 ? nlxp : 1);
    } else {
        lua_pushnumber(L, 1);
    }
    return 1;
}

// GetXPExhaustion() → rested XP pool remaining (nil if none)
static int lua_GetXPExhaustion(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { return luaReturnNil(L); }
    uint32_t rested = gh->getPlayerRestedXp();
    if (rested > 0) lua_pushnumber(L, rested);
    else lua_pushnil(L);
    return 1;
}

/// GetWatchedFactionInfo() → name, standingID, barMin, barMax, barValue.
///
/// The reputation bar under the experience bar, which draws nothing at all
/// without this. Returns nothing when no faction is being watched, which is
/// what MainMenuBar_UpdateExperienceBars checks for before showing the bar.
///
/// The bounds are the rank's own, not the whole scale: WoW's bar fills from
/// the bottom of the current rank to the top of it, so a character halfway
/// through Honored shows a half-full bar rather than one two-thirds along.
static int lua_GetWatchedFactionInfo(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) return 0;
    const uint32_t factionId = gh->getWatchedFactionId();
    if (factionId == 0) return 0;
    const auto& standings = gh->getFactionStandings();
    auto it = standings.find(factionId);
    if (it == standings.end()) return 0;
    const int32_t standing = it->second;

    // Blizzard's own thresholds, and the standing id that goes with each. Hated
    // is 1 and Exalted is 8, which is what the interface indexes FACTION_BAR_
    // COLORS and the standing names by.
    static const struct { int32_t min, max; } kRanks[8] = {
        {-42000, -6000}, {-6000, -3000}, {-3000, 0}, {0, 3000},
        {3000, 9000}, {9000, 21000}, {21000, 42000}, {42000, 43000},
    };
    int rank = 7;
    for (int i = 0; i < 8; ++i) {
        if (standing < kRanks[i].max) { rank = i; break; }
    }

    lua_pushstring(L, gh->getFactionNamePublic(factionId).c_str());
    lua_pushnumber(L, rank + 1);
    lua_pushnumber(L, kRanks[rank].min);
    lua_pushnumber(L, kRanks[rank].max);
    lua_pushnumber(L, standing);
    lua_pushnumber(L, factionId);
    return 6;
}

/// GetNumBagSlots() → the four bags beside the backpack. A constant in every
/// expansion this client speaks, but the interface reads it rather than
/// assuming, and arithmetic on nil is what it gets otherwise.
static int lua_GetNumBagSlots(lua_State* L) {
    lua_pushnumber(L, 4);
    return 1;
}

/// GetMirrorTimerProgress(timer) → milliseconds left on breath, fatigue or
/// feign death. None of the three is tracked yet, and the honest answer is
/// zero rather than a stub that reads as a timer running.
/// GetMirrorTimerProgress(timer) → milliseconds left on that timer.
///
/// It answered a constant zero, and MirrorTimerFrame_OnUpdate divides it by a
/// thousand and calls SetValue with the result on every frame - so the breath
/// bar was reset to empty as fast as it could be drawn. An empty frame with a
/// label was the whole of what it could ever show.
///
/// Named, not numbered: WoW passes the timer's name through the event and
/// FrameXML hands the same string back here.
static int lua_GetMirrorTimerProgress(lua_State* L) {
    auto* gh = getGameHandler(L);
    std::string name(luaL_optstring(L, 1, ""));
    toLowerInPlace(name);
    int type = -1;
    if (name == "exhaustion") type = 0;
    else if (name == "breath") type = 1;
    else if (name == "death") type = 2;
    if (!gh || type < 0) { lua_pushnumber(L, 0); return 1; }
    lua_pushnumber(L, gh->getMirrorTimer(type).value);
    return 1;
}

/// GetTimeToWellRested() → seconds until fully rested, or nil.
///
/// Confirmed called rather than guessed at: it turned up in the missing-API
/// report from a real session, read by the experience bar's tooltip. Rest
/// accrual is not modelled, and nil is what WoW itself answers when there is
/// no timer to report - which is the branch the tooltip already handles.
static int lua_GetTimeToWellRested(lua_State* L) {
    return luaReturnNil(L);
}


// ── Character sheet ────────────────────────────────────────────────────────
//
// PaperDollFrame reads about forty functions and does arithmetic with nearly
// all of them, so a missing one is not a blank field - it is "attempt to
// perform arithmetic on a nil value" and the rest of the panel never draws.
// The server sends most of what these want; where it does not, the honest
// answer is a zero of the right shape rather than nothing.

/// UnitAttackPower(unit) → base, positive buff, negative buff.
///
/// The server sends one effective number, not the three parts, so it is
/// reported as all base and no buffs. The sheet adds them together, which
/// gives the right total either way.
static int lua_UnitAttackPower(lua_State* L) {
    auto* gh = getGameHandler(L);
    // Whose. PaperDollFrame_SetAttackPower is shared and the pet tab calls it
    // with "Pet", so the pet's line showed the hunter's attack power.
    std::string who(luaL_optstring(L, 1, "player"));
    toLowerInPlace(who);
    if (who == "pet") {
        const int32_t petAp = gh ? gh->getPetAttackPower() : 0;
        lua_pushnumber(L, petAp > 0 ? petAp : 0);
        lua_pushnumber(L, 0);
        lua_pushnumber(L, 0);
        return 3;
    }
    const int32_t ap = gh ? gh->getMeleeAttackPower() : -1;
    lua_pushnumber(L, ap > 0 ? ap : 0);
    lua_pushnumber(L, 0);
    lua_pushnumber(L, 0);
    return 3;
}
static int lua_UnitRangedAttackPower(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int32_t ap = gh ? gh->getRangedAttackPower() : -1;
    lua_pushnumber(L, ap > 0 ? ap : 0);
    lua_pushnumber(L, 0);
    lua_pushnumber(L, 0);
    return 3;
}

/// UnitDefense(unit) → base, modifier. Base defense skill is five per level -
/// the cap a character sits at in practice, which is all the server lets us
/// infer since it does not send the trained skill. The modifier is the defense
/// skill granted by defense rating: AzerothCore takes int32(GetRatingBonusValue(
/// CR_DEFENSE_SKILL)) and each point is worth 0.04% avoidance. It was a flat
/// zero, so a geared tank's defense showed only the level cap with no sign of
/// the defence rating that pushes it to the crit-immunity threshold.
static int lua_UnitDefense(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int level = gh ? static_cast<int>(gh->getPlayerLevel()) : 1;
    constexpr int kCrDefenseSkill = 1;  // 0-based CombatRating enum
    const int modifier = gh ? static_cast<int>(gh->getCombatRatingBonus(kCrDefenseSkill)) : 0;
    lua_pushnumber(L, level * 5);
    lua_pushnumber(L, modifier);
    return 2;
}

/// UnitAttackSpeed(unit) → main hand, off hand, in seconds. Nothing here
/// tracks weapon speed yet; two seconds is a one-handed weapon and keeps the
/// damage-per-second division that follows from dividing by zero.
/// UnitAttackSpeed(unit) → mainHandSpeed, offHandSpeed.
///
/// The off-hand speed is nil when nothing is in that hand, and that is what
/// the character sheet branches on: PaperDollFrame_SetAttackSpeed does
/// `if ( offhandSpeed )` and then prints a two-handed line. Answering a speed
/// unconditionally claimed an off-hand weapon for everyone, and the line it
/// then built concatenated a value nothing had filled in.
static int lua_UnitAttackSpeed(lua_State* L) {
    auto* gh = getGameHandler(L);
    const char* uid = luaL_optstring(L, 1, "player");
    std::string u(uid);
    toLowerInPlace(u);
    // The comment above used to say nothing tracked weapon speed. The equipped
    // item carries delayMs and always has, so both hands answer from the
    // weapon rather than from a flat two seconds - which every damage-per-
    // second figure on the sheet is divided by.
    double mainSpeed = 2.0;      // unarmed, which is what WoW uses bare-handed
    double offSpeed = 0.0;
    bool hasOffHand = false;
    if (gh && u == "player") {
        const auto& inv = gh->getInventory();
        const auto& mh = inv.getEquipSlot(game::EquipSlot::MAIN_HAND);
        if (!mh.empty() && mh.item.delayMs > 0) mainSpeed = mh.item.delayMs / 1000.0;
        const auto& oh = inv.getEquipSlot(game::EquipSlot::OFF_HAND);
        hasOffHand = !oh.empty();
        if (hasOffHand) offSpeed = oh.item.delayMs > 0 ? oh.item.delayMs / 1000.0 : 2.0;
    }
    lua_pushnumber(L, mainSpeed);
    if (hasOffHand) lua_pushnumber(L, offSpeed);
    else            lua_pushnil(L);
    return 2;
}

/// UnitDamage(unit) → min, max, off-hand min, off-hand max, positive bonus,
/// negative bonus, percent.
///
/// The comment here used to say weapon damage was not tracked, so it reported
/// the attack-power contribution alone against a two-second swing. The
/// equipped weapon carries its damage range and its speed, so the figure is
/// the weapon's own damage plus what attack power adds over one swing -
/// attack power divided by fourteen is damage per second, times the speed.
static int lua_UnitDamage(lua_State* L) {
    auto* gh = getGameHandler(L);
    // The pet's damage comes off its own fields rather than from a weapon it
    // does not carry, so it is answered before any of the equipment below.
    std::string who(luaL_optstring(L, 1, "player"));
    toLowerInPlace(who);
    if (who == "pet") {
        const double lo = gh ? gh->getPetMinDamage() : 0.0;
        const double hi = gh ? gh->getPetMaxDamage() : 0.0;
        lua_pushnumber(L, lo);   // minDamage
        lua_pushnumber(L, hi);   // maxDamage
        lua_pushnumber(L, 0.0);  // minOffHand
        lua_pushnumber(L, 0.0);  // maxOffHand
        lua_pushnumber(L, 0.0);  // physical bonus, positive
        lua_pushnumber(L, 0.0);  // ...and negative
        lua_pushnumber(L, 1.0);  // damage percent
        return 7;
    }
    const int32_t ap = gh ? gh->getMeleeAttackPower() : -1;
    double baseMin = 1.0, baseMax = 2.0;   // bare hands
    double speed = 2.0;
    double offMin = 0.0, offMax = 0.0;
    if (gh) {
        const auto& inv = gh->getInventory();
        const auto& mh = inv.getEquipSlot(game::EquipSlot::MAIN_HAND);
        if (!mh.empty() && mh.item.damageMax > 0.0f) {
            baseMin = mh.item.damageMin;
            baseMax = mh.item.damageMax;
            if (mh.item.delayMs > 0) speed = mh.item.delayMs / 1000.0;
        }
        const auto& oh = inv.getEquipSlot(game::EquipSlot::OFF_HAND);
        if (!oh.empty() && oh.item.damageMax > 0.0f) {
            offMin = oh.item.damageMin;
            offMax = oh.item.damageMax;
        }
    }
    const double apSwing = (ap > 0) ? (ap / 14.0) * speed : 0.0;
    lua_pushnumber(L, baseMin + apSwing);
    lua_pushnumber(L, baseMax + apSwing);
    lua_pushnumber(L, offMin > 0.0 ? offMin + apSwing / 2.0 : 0.0);
    lua_pushnumber(L, offMax > 0.0 ? offMax + apSwing / 2.0 : 0.0);
    lua_pushnumber(L, 0);
    lua_pushnumber(L, 0);
    lua_pushnumber(L, 1.0);
    return 7;
}

/// UnitRangedDamage(unit) → speed, min, max, positive, negative, percent.
static int lua_UnitRangedDamage(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int32_t ap = gh ? gh->getRangedAttackPower() : -1;
    const double swing = (ap > 0) ? (ap / 14.0) * 2.8 : 0.0;
    lua_pushnumber(L, 2.8);
    lua_pushnumber(L, swing);
    lua_pushnumber(L, swing);
    lua_pushnumber(L, 0);
    lua_pushnumber(L, 0);
    lua_pushnumber(L, 1.0);
    return 6;
}

/// UnitRangedAttack(unit) → skill, modifier. As with defense, five per level.
static int lua_UnitRangedAttack(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int level = gh ? static_cast<int>(gh->getPlayerLevel()) : 1;
    lua_pushnumber(L, level * 5);
    lua_pushnumber(L, 0);
    return 2;
}

/// UnitAttackBothHands(unit) → main skill, main modifier, off skill, off
/// modifier.
static int lua_UnitAttackBothHands(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int level = gh ? static_cast<int>(gh->getPlayerLevel()) : 1;
    lua_pushnumber(L, level * 5);
    lua_pushnumber(L, 0);
    lua_pushnumber(L, level * 5);
    lua_pushnumber(L, 0);
    return 4;
}

/// GetManaRegen() → while not casting, while casting. Both per second; the
/// paperdoll multiplies by five to show mana-per-5-seconds. The server sends
/// both figures already computed (UNIT_FIELD_POWER_REGEN_FLAT_MODIFIER and its
/// interrupted twin), so this reads them rather than modelling the spirit and
/// intellect formula - it was a flat zero, so every caster's mana regen read 0.
static int lua_GetManaRegen(lua_State* L) {
    auto* gh = getGameHandler(L);
    lua_pushnumber(L, gh ? gh->getManaRegen() : 0.0);
    lua_pushnumber(L, gh ? gh->getManaRegenCasting() : 0.0);
    return 2;
}

/// The several figures the sheet works out from a rating or a stat. None of
/// the conversion formulae are modelled - they are level-dependent tables the
/// server does not send - and the percentages that matter are reported
/// directly by GetCritChance, GetDodgeChance and their kind, which do come
/// from the server. Zero here loses a tooltip breakdown, not a number anyone
/// plays by.
static int lua_ZeroPercent(lua_State* L) {
    lua_pushnumber(L, 0);
    return 1;
}

// GetCritChanceFromAgility([unit]) → the melee crit percent the player's Agility
// gives, from the combat game tables. Was a flat zero, so the crit stat's
// flyout read "0.00% from Agility" for every character.
static int lua_GetCritChanceFromAgility(lua_State* L) {
    auto* gh = getGameHandler(L);
    lua_pushnumber(L, gh ? gh->getMeleeCritFromAgility() : 0.0);
    return 1;
}

// GetSpellCritChanceFromIntellect([unit]) → the spell crit percent Intellect
// gives, from the spell-crit game tables. Was a flat zero.
static int lua_GetSpellCritChanceFromIntellect(lua_State* L) {
    auto* gh = getGameHandler(L);
    lua_pushnumber(L, gh ? gh->getSpellCritFromIntellect() : 0.0);
    return 1;
}

// GetCombatRatingBonus(ratingIndex) → the percent that combat rating grants.
// The paperdoll reads it for dodge, parry, haste, defense, weapon skill and the
// other rating-based stats; it was a flat zero for all of them.
static int lua_GetCombatRatingBonus(lua_State* L) {
    auto* gh = getGameHandler(L);
    // Same 1-based CR_* → 0-based table convention as GetCombatRating above.
    const int cr = static_cast<int>(luaL_optnumber(L, 1, 0)) - 1;
    lua_pushnumber(L, gh ? gh->getCombatRatingBonus(cr) : 0.0);
    return 1;
}

// GetArmorPenetration() → the percent of the target's armour the player ignores.
// The server stores armour penetration as a rating in the combat-rating field
// (Player::UpdateArmorPenetration writes PLAYER_FIELD_COMBAT_RATING_1 +
// CR_ARMOR_PENETRATION), and the paperdoll shows the rating's converted percent
// beside it - so this is the CR_ARMOR_PENETRATION rating bonus, index 24 in the
// 0-based internal scheme. Takes no unit; the flyout only asks about the player.
static int lua_GetArmorPenetration(lua_State* L) {
    auto* gh = getGameHandler(L);
    constexpr int kCrArmorPenetration = 24;  // 0-based CombatRating enum
    lua_pushnumber(L, gh ? gh->getCombatRatingBonus(kCrArmorPenetration) : 0.0);
    return 1;
}

// Health / mana regenerated per second from Spirit - the pair the Spirit stat
// flyout shows. Were flat zeros. Only the player's Spirit and class are tracked,
// so any other unit answers zero rather than the player's value under its name.
static int lua_GetUnitHealthRegenRateFromSpirit(lua_State* L) {
    auto* gh = getGameHandler(L);
    std::string unit(luaL_optstring(L, 1, "player"));
    toLowerInPlace(unit);
    lua_pushnumber(L, (gh && unit == "player") ? gh->getHealthRegenFromSpirit() : 0.0);
    return 1;
}
static int lua_GetUnitManaRegenRateFromSpirit(lua_State* L) {
    auto* gh = getGameHandler(L);
    std::string unit(luaL_optstring(L, 1, "player"));
    toLowerInPlace(unit);
    lua_pushnumber(L, (gh && unit == "player") ? gh->getManaRegenFromSpirit() : 0.0);
    return 1;
}

/// GetUnitMaxHealthModifier(unit) → the multiplier on maximum health. One,
/// because nothing here modifies it - and one rather than zero because the
/// sheet multiplies by it.
static int lua_GetUnitMaxHealthModifier(lua_State* L) {
    lua_pushnumber(L, 1.0);
    return 1;
}

/// GetInventoryItemCooldown(unit, slot) → start, duration, enabled. All zero
/// is "nothing on cooldown", which is what the sheet checks for before doing
/// arithmetic with the first two.
/// GetInventoryItemCooldown(unit, slot) → start, duration, enable.
///
/// The same as the container one and for the same reason: an equipped trinket
/// on cooldown drew nothing on the paperdoll, which is handed over.
static int lua_GetInventoryItemCooldown(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int slot = static_cast<int>(luaL_optnumber(L, 2, 0));
    double start = 0.0, duration = 0.0;
    if (gh && slot >= 1 && slot <= 19) {
        const auto& sl = gh->getInventory().getEquipSlot(
            static_cast<game::EquipSlot>(slot - 1));
        if (!sl.empty()) itemUseCooldown(gh, sl.item.itemId, start, duration);
    }
    lua_pushnumber(L, start);
    lua_pushnumber(L, duration);
    lua_pushnumber(L, 1);
    return 3;
}

// SetPortraitTexture(texture, unit) - put a unit's face in a texture.
//
// The player's and the target's, each rendered to an offscreen image of its
// own. Any other unit leaves the texture as it was and clears every claim on
// it, so a portrait frame reused for a party member does not keep showing
// whichever of the two it last held.
//
// The texture is remembered rather than filled here: the handle is rebuilt
// whenever the portrait's render target is, so it has to be assigned every
// frame, which the render loop does for everything on this list.
static int lua_SetPortraitTexture(lua_State* L) {
    auto* tree = getWidgetTree(L);
    if (!tree || !lua_istable(L, 1)) return 0;
    // The widget id the frame table carries. widgetIdOf lives in lua_engine.cpp
    // and is not declared anywhere this file can see it.
    lua_getfield(L, 1, "__wid");
    const uint32_t id = static_cast<uint32_t>(lua_tointeger(L, -1));
    lua_pop(L, 1);
    if (id == 0) return 0;

    std::string unit(luaL_optstring(L, 2, ""));
    toLowerInPlace(unit);
    // Only the units this client can build a face for. Anything else releases
    // the texture rather than claiming it, so a portrait frame reused for a
    // unit with no picture does not keep the last one it had.
    const bool answerable =
        (unit == "player" || unit == "target" || unit == "pet" ||
         unit == "focus" || unit == "npc" || unit == "questnpc" ||
         (unit.rfind("party", 0) == 0 && unit.size() == 6 &&
          unit[5] >= '1' && unit[5] <= '4'));
    tree->setPortraitUnit(id, answerable ? unit : std::string());
    return 0;
}

// --- What the barber shop calls the two things it can change ---
//
// These name a category rather than describing it, and the barber builds a
// global's name out of the answer:
//
//     BarberShopFrameSelector1Category:SetText(_G["HAIR_"..GetHairCustomization().."_STYLE"])
//
// Concatenating nil raises, and this runs from BarberShop_OnLoad - which the
// client reaches the moment a player sits down, because BARBER_SHOP_OPEN is
// fired and the interface answers it by loading the barber addon. So sitting
// in the chair took the addon down as it loaded.
//
// The hair category is NORMAL for everyone but tauren, who have horns where
// other races have hair - the barber offers them "Horn Style" and "Horn Color",
// and HAIR_HORNS_STYLE and HAIR_HORNS_COLOR are both there in globalstrings
// beside the NORMAL pair. It is the only race that differs, which is why the
// facial-hair function below has a switch and this has an if.
static int lua_GetHairCustomization(lua_State* L) {
    auto* gh = getGameHandler(L);
    const bool tauren = gh &&
        static_cast<game::Race>(gh->getPlayerRace()) == game::Race::TAUREN;
    lua_pushstring(L, tauren ? "HORNS" : "NORMAL");
    return 1;
}

/// The facial category, which genuinely differs by race - a troll's is tusks
/// and a night elf's is markings, and calling either "hair" reads as a mistake
/// rather than as a shortcut. These are the game's own categories; anything
/// unlisted falls back to NORMAL, which is what a beard is.
static int lua_GetFacialHairCustomization(lua_State* L) {
    auto* gh = getGameHandler(L);
    const char* category = "NORMAL";
    if (gh) {
        switch (static_cast<game::Race>(gh->getPlayerRace())) {
            case game::Race::NIGHT_ELF: category = "MARKINGS"; break;
            case game::Race::UNDEAD:    category = "FEATURES"; break;
            case game::Race::TAUREN:    category = "HORNS";    break;
            case game::Race::TROLL:     category = "TUSKS";    break;
            case game::Race::BLOOD_ELF: category = "EARRINGS"; break;
            default: break;
        }
    }
    lua_pushstring(L, category);
    return 1;
}

/// CanAlterSkin() - whether the barber offers a fourth selector for skin.
///
/// False, and not merely for want of data: answering yes would put a selector
/// on screen that this client cannot fill, and the barber tests every selector
/// it has drawn before it will let the player buy anything.
static int lua_CanAlterSkin(lua_State* L) {
    lua_pushboolean(L, 0);
    return 1;
}

// --- The barber's selectors, its price and its two exits ---
//
// The style lists and the originals used to be built inside this client's own
// barber window, so with the panel handed over nothing had built them. They
// come from WindowManager through LuaServices now, from a version that runs
// whether or not this client is drawing the chair.

/// GetBarberShopStyleInfo(selector) → name, category, cost, isCurrent
///
/// blizzard_barbershopui.lua reads the first and the fourth, and reads the
/// fourth through select(4, ...) - so all four have to be there for the index
/// to land on the right one.
static int lua_GetBarberShopStyleInfo(lua_State* L) {
    const int selector = static_cast<int>(luaL_optinteger(L, 1, 0));
    auto* svc = getLuaServices(L);
    std::string name;
    bool isCurrent = false;
    if (!svc || !svc->getBarberStyleInfo ||
        !svc->getBarberStyleInfo(selector, name, isCurrent)) {
        return 0;
    }
    lua_pushstring(L, name.c_str());
    lua_pushstring(L, "");     // category, which this client does not name
    lua_pushinteger(L, 0);     // per-selector cost; the total is the one asked for
    lua_pushboolean(L, isCurrent ? 1 : 0);
    return 4;
}

/// SetNextBarberShopStyle(selector, direction) - the arrows beside a selector.
///
/// The forward arrow passes 1 and the back arrow passes nothing at all
/// (blizzard_barbershopui.xml:31 and :56), so an absent argument means back
/// rather than a missing parameter.
static int lua_SetNextBarberShopStyle(lua_State* L) {
    const int selector = static_cast<int>(luaL_optinteger(L, 1, 0));
    const int direction = lua_isnoneornil(L, 2) ? -1 : 1;
    if (auto* svc = getLuaServices(L); svc && svc->setNextBarberStyle) {
        svc->setNextBarberStyle(selector, direction);
    }
    return 0;
}

/// GetBarberShopTotalCost() → copper for everything currently changed.
static int lua_GetBarberShopTotalCost(lua_State* L) {
    auto* svc = getLuaServices(L);
    lua_pushinteger(L, svc && svc->getBarberTotalCost
                           ? static_cast<lua_Integer>(svc->getBarberTotalCost()) : 0);
    return 1;
}

/// BarberShopReset() - put every selector back to what the character wears.
static int lua_BarberShopReset(lua_State* L) {
    if (auto* svc = getLuaServices(L); svc && svc->barberReset) svc->barberReset();
    return 0;
}

/// CancelBarberShop() - leave the chair without buying anything.
static int lua_CancelBarberShop(lua_State* L) {
    if (auto* gh = getGameHandler(L)) gh->closeBarberShop();
    return 0;
}

/// GetMaxCombatRatingBonus(rating) → the cap on what a rating can give. Zero
/// would read as "capped at nothing", so this answers with a hundred percent,
/// which is no cap in any practical sense.
static int lua_GetMaxCombatRatingBonus(lua_State* L) {
    lua_pushnumber(L, 100);
    return 1;
}

// GetRestState() → 1 = normal, 2 = rested
/// GetRestState() → stateID, stateName, xpMultiplier.
///
/// One is rested and two is normal, which is the way round WoW numbers them -
/// this answered the opposite. MainMenuBar multiplies by the third return the
/// line after reading it, so returning only the id was arithmetic on nil every
/// time the cursor crossed the experience bar.
static int lua_GetRestState(lua_State* L) {
    auto* gh = getGameHandler(L);
    const bool rested = gh && gh->isPlayerResting();
    lua_pushnumber(L, rested ? 1 : 2);
    lua_pushstring(L, rested ? "Rested" : "Normal");
    lua_pushnumber(L, rested ? 1.5 : 1.0);
    return 3;
}

// --- Quest Log API ---

static int lua_UnitAffectingCombat(lua_State* L) {
    const char* uid = luaL_optstring(L, 1, "player");
    auto* gh = getGameHandler(L);
    if (!gh) { return luaReturnFalse(L); }
    std::string uidStr(uid);
    toLowerInPlace(uidStr);
    if (uidStr == "player") {
        lua_pushboolean(L, gh->isInCombat());
    } else {
        // Check UNIT_FLAG_IN_COMBAT (0x00080000) in UNIT_FIELD_FLAGS
        uint64_t guid = resolveUnitGuid(gh, uidStr);
        bool inCombat = false;
        if (guid != 0) {
            auto entity = gh->getEntityManager().getEntity(guid);
            if (entity) {
                uint32_t flags = entity->getField(
                    game::fieldIndex(game::UF::UNIT_FIELD_FLAGS));
                inCombat = (flags & 0x00080000) != 0; // UNIT_FLAG_IN_COMBAT
            }
        }
        lua_pushboolean(L, inCombat);
    }
    return 1;
}



// HasFullControl() - whether the player is in command of their own character
//
// Read as `not HasFullControl()` to grey out the right-click menu, so an absent
// answer disabled duelling, trading and inviting permanently. Nothing here
// charms, fears or confuses anyone, so control is never lost and saying so is
// the truthful answer rather than a convenient one.
// GetPetSpellBonusDamage() → the spell power a pet inherits
//
// The pet tab formats this with %d before testing it, and %d against nil raises
// on the spot - on a tab of the character frame, which is drawn by default.
// Nothing here works out what a pet inherits, and zero is the honest figure
// rather than a guess at a scaling rule.
static int lua_GetPetSpellBonusDamage(lua_State* L) {
    lua_pushnumber(L, 0);
    return 1;
}

static int lua_HasFullControl(lua_State* L) {
    lua_pushboolean(L, 1);
    return 1;
}

// --- Dying, and getting back up ---
//
// The death popup already appears; its buttons had nothing behind them, so a
// player could be told they had died and be unable to do anything about it.

// RepopMe() - release the spirit and run back

// RetrieveCorpse() - resurrect where the body is
static int lua_RetrieveCorpse(lua_State* L) {
    if (auto* gh = getGameHandler(L)) gh->reclaimCorpse();
    return 0;
}

// Dismount() - what /dismount does
static int lua_Dismount(lua_State* L) {
    if (auto* gh = getGameHandler(L)) gh->dismount();
    return 0;
}

// StartAttack([unit]) / StopAttack() - swing at what is targeted
//
// The interface passes a unit only to attack something other than the current
// target, which this client cannot do without changing target first, so the
// argument is read and the current target used.
static int lua_StartAttack(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) return 0;
    const uint64_t target = gh->getTargetGuid();
    if (target != 0) gh->startAutoAttack(target);
    return 0;
}

static int lua_StopAttack(lua_State* L) {
    if (auto* gh = getGameHandler(L)) gh->stopAutoAttack();
    return 0;
}

// GetReleaseTimeRemaining() → milliseconds before the corpse releases itself,
// or -1 when nothing will force it
//
// UIParent asks this the moment the player dies, as `> 0 or == -1`, so an
// absent answer is an error on every death rather than a popup that does not
// appear. Nothing here counts down to a forced release, and -1 is the game's
// own way of saying so.

static int lua_GetNumRaidMembers(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh || !gh->isInGroup()) { return luaReturnZero(L); }
    const auto& pd = gh->getPartyData();
    lua_pushnumber(L, (pd.groupType == 1) ? pd.memberCount : 0);
    return 1;
}

static int lua_GetNumPartyMembers(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh || !gh->isInGroup()) { return luaReturnZero(L); }
    const auto& pd = gh->getPartyData();
    // In party (not raid), count excludes self
    int count = (pd.groupType == 0) ? static_cast<int>(pd.memberCount) : 0;
    // memberCount includes self on some servers, subtract 1 if needed
    if (count > 0) count = std::max(0, count - 1);
    lua_pushnumber(L, count);
    return 1;
}

static int lua_UnitInParty(lua_State* L) {
    const char* uid = luaL_optstring(L, 1, "player");
    auto* gh = getGameHandler(L);
    if (!gh) { return luaReturnFalse(L); }
    std::string uidStr(uid);
    toLowerInPlace(uidStr);
    if (uidStr == "player") {
        lua_pushboolean(L, gh->isInGroup());
    } else {
        uint64_t guid = resolveUnitGuid(gh, uidStr);
        if (guid == 0) { return luaReturnFalse(L); }
        const auto& pd = gh->getPartyData();
        bool found = false;
        for (const auto& m : pd.members) {
            if (m.guid == guid) { found = true; break; }
        }
        lua_pushboolean(L, found);
    }
    return 1;
}

static int lua_UnitInRaid(lua_State* L) {
    const char* uid = luaL_optstring(L, 1, "player");
    auto* gh = getGameHandler(L);
    if (!gh) { return luaReturnFalse(L); }
    std::string uidStr(uid);
    toLowerInPlace(uidStr);
    const auto& pd = gh->getPartyData();
    if (pd.groupType != 1) { return luaReturnFalse(L); }
    if (uidStr == "player") {
        lua_pushboolean(L, 1);
        return 1;
    }
    uint64_t guid = resolveUnitGuid(gh, uidStr);
    bool found = false;
    for (const auto& m : pd.members) {
        if (m.guid == guid) { found = true; break; }
    }
    lua_pushboolean(L, found);
    return 1;
}

// GetRaidRosterInfo(index) → name, rank, subgroup, level, class, fileName, zone, online, isDead, role, isML
static int lua_GetRaidRosterInfo(lua_State* L) {
    auto* gh = getGameHandler(L);
    int index = static_cast<int>(luaL_checknumber(L, 1));
    if (!gh || index < 1) { return luaReturnNil(L); }
    const auto& pd = gh->getPartyData();
    if (index > static_cast<int>(pd.members.size())) { return luaReturnNil(L); }
    const auto& m = pd.members[index - 1];
    lua_pushstring(L, m.name.c_str());       // name
    lua_pushnumber(L, m.guid == pd.leaderGuid ? 2 : (m.flags & 0x01 ? 1 : 0)); // rank (0=member, 1=assist, 2=leader)
    lua_pushnumber(L, m.subGroup + 1);       // subgroup (1-indexed)
    lua_pushnumber(L, m.level);              // level
    // Class: resolve from entity if available
    std::string className = "Unknown";
    auto entity = gh->getEntityManager().getEntity(m.guid);
    if (entity) {
        uint32_t bytes0 = entity->getField(game::fieldIndex(game::UF::UNIT_FIELD_BYTES_0));
        uint8_t classId = static_cast<uint8_t>((bytes0 >> 8) & 0xFF);
        if (classId > 0 && classId < 12) className = kLuaClasses[classId];
    }
    lua_pushstring(L, className.c_str());    // class (localized)
    lua_pushstring(L, className.c_str());    // fileName
    lua_pushstring(L, "");                   // zone
    lua_pushboolean(L, m.isOnline);          // online
    lua_pushboolean(L, m.curHealth == 0);    // isDead
    lua_pushstring(L, "NONE");               // role
    lua_pushboolean(L, pd.looterGuid == m.guid ? 1 : 0); // isML
    return 11;
}

// GetThreatStatusColor(statusIndex) → r, g, b
static int lua_GetThreatStatusColor(lua_State* L) {
    int status = static_cast<int>(luaL_optnumber(L, 1, 0));
    switch (status) {
        case 0: lua_pushnumber(L, 0.69f); lua_pushnumber(L, 0.69f); lua_pushnumber(L, 0.69f); break; // gray (no threat)
        case 1: lua_pushnumber(L, 1.0f);  lua_pushnumber(L, 1.0f);  lua_pushnumber(L, 0.47f); break; // yellow (threat)
        case 2: lua_pushnumber(L, 1.0f);  lua_pushnumber(L, 0.6f);  lua_pushnumber(L, 0.0f);  break; // orange (high threat)
        case 3: lua_pushnumber(L, 1.0f);  lua_pushnumber(L, 0.0f);  lua_pushnumber(L, 0.0f);  break; // red (tanking)
        default: lua_pushnumber(L, 1.0f); lua_pushnumber(L, 1.0f);  lua_pushnumber(L, 1.0f);  break;
    }
    return 3;
}

// GetReadyCheckStatus(unit) → status string
static int lua_GetReadyCheckStatus(lua_State* L) {
    (void)L;
    lua_pushnil(L); // No ready check in progress
    return 1;
}

// RegisterUnitWatch / UnregisterUnitWatch - secure unit frame stubs
static int lua_RegisterUnitWatch(lua_State* L) { (void)L; return 0; }
static int lua_UnregisterUnitWatch(lua_State* L) { (void)L; return 0; }

static int lua_UnitIsUnit(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { return luaReturnFalse(L); }
    const char* uid1 = luaL_checkstring(L, 1);
    const char* uid2 = luaL_checkstring(L, 2);
    std::string u1(uid1), u2(uid2);
    toLowerInPlace(u1);
    toLowerInPlace(u2);
    uint64_t g1 = resolveUnitGuid(gh, u1);
    uint64_t g2 = resolveUnitGuid(gh, u2);
    lua_pushboolean(L, g1 != 0 && g1 == g2);
    return 1;
}

/// UnitIsFriend(unit1, unit2) - is unit2 friendly to unit1.
///
/// Two units, and the answer is about the *second*. This read only the first,
/// which every caller in the interface passes as "player" - and the player is
/// never hostile, so it answered true for whatever was being asked about.
/// targetframe.lua colours the name from it, picks the debuff layout from it
/// and filters with it, so every target read as a friend.
///
/// Hostility is modelled relative to the player, so the unit that carries the
/// answer is whichever of the pair is *not* the player - and the interface
/// passes both orders: UnitIsFriend("player", self.unit) beside
/// UnitIsEnemy(self.unit, "player"). Keying on position would be right for one
/// of them and wrong for the other.
static game::Unit* unitAskedAbout(lua_State* L) {
    auto* gh = getGameHandler(L);
    const char* a = luaL_optstring(L, 1, "player");
    const char* b = luaL_optstring(L, 2, nullptr);
    if (!b) return resolveUnit(L, a);
    if (!gh) return resolveUnit(L, b);
    std::string ua(a), ub(b);
    toLowerInPlace(ua);
    toLowerInPlace(ub);
    const uint64_t player = gh->getPlayerGuid();
    if (resolveUnitGuid(gh, ua) == player) return resolveUnit(L, b);
    if (resolveUnitGuid(gh, ub) == player) return resolveUnit(L, a);
    // Neither is the player: answered about the second, which is the one the
    // question is grammatically about.
    return resolveUnit(L, b);
}

static int lua_UnitIsFriend(lua_State* L) {
    auto* unit = unitAskedAbout(L);
    lua_pushboolean(L, unit && !unit->isHostile());
    return 1;
}

/// UnitIsEnemy(unit1, unit2) - is unit2 hostile to unit1. The same shape as
/// UnitIsFriend above, and it was wrong the same way: asked about the player it
/// answered false for every enemy.
static int lua_UnitIsEnemy(lua_State* L) {
    auto* unit = unitAskedAbout(L);
    lua_pushboolean(L, unit && unit->isHostile());
    return 1;
}

static int lua_UnitCreatureType(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { lua_pushstring(L, "Unknown"); return 1; }
    const char* uid = luaL_optstring(L, 1, "target");
    std::string uidStr(uid);
    toLowerInPlace(uidStr);
    uint64_t guid = resolveUnitGuid(gh, uidStr);
    if (guid == 0) { lua_pushstring(L, "Unknown"); return 1; }
    auto entity = gh->getEntityManager().getEntity(guid);
    if (!entity) { lua_pushstring(L, "Unknown"); return 1; }
    // Player units are always "Humanoid"
    if (entity->getType() == game::ObjectType::PLAYER) {
        lua_pushstring(L, "Humanoid");
        return 1;
    }
    auto unit = std::dynamic_pointer_cast<game::Unit>(entity);
    if (!unit) { lua_pushstring(L, "Unknown"); return 1; }
    uint32_t ct = gh->getCreatureType(unit->getEntry());
    static constexpr const char* kTypes[] = {
        "Unknown", "Beast", "Dragonkin", "Demon", "Elemental",
        "Giant", "Undead", "Humanoid", "Critter", "Mechanical",
        "Not specified", "Totem", "Non-combat Pet", "Gas Cloud"
    };
    lua_pushstring(L, (ct < 14) ? kTypes[ct] : "Unknown");
    return 1;
}

// GetPlayerInfoByGUID(guid) → localizedClass, englishClass, localizedRace, englishRace, sex, name, realm
static int lua_GetPlayerInfoByGUID(lua_State* L) {
    auto* gh = getGameHandler(L);
    const char* guidStr = luaL_checkstring(L, 1);
    if (!gh || !guidStr) {
        for (int i = 0; i < 7; i++) lua_pushnil(L);
        return 7;
    }
    // Parse hex GUID string "0x0000000000000001"
    uint64_t guid = 0;
    if (guidStr[0] == '0' && (guidStr[1] == 'x' || guidStr[1] == 'X'))
        guid = strtoull(guidStr + 2, nullptr, 16);
    else
        guid = strtoull(guidStr, nullptr, 16);

    if (guid == 0) { for (int i = 0; i < 7; i++) lua_pushnil(L); return 7; }

    // Look up entity name
    std::string name = gh->lookupName(guid);
    if (name.empty() && guid == gh->getPlayerGuid()) {
        const auto& chars = gh->getCharacters();
        for (const auto& c : chars)
            if (c.guid == guid) { name = c.name; break; }
    }

    // For player GUID, return class/race if it's the local player
    const char* className = "Unknown";
    const char* raceName = "Unknown";
    if (guid == gh->getPlayerGuid()) {
        uint8_t cid = gh->getPlayerClass();
        uint8_t rid = gh->getPlayerRace();
        if (cid < 12) className = kLuaClasses[cid];
        if (rid < 12) raceName = kLuaRaces[rid];
    }

    lua_pushstring(L, className);  // 1: localizedClass
    lua_pushstring(L, className);  // 2: englishClass
    lua_pushstring(L, raceName);   // 3: localizedRace
    lua_pushstring(L, raceName);   // 4: englishRace
    lua_pushnumber(L, 0);          // 5: sex (0=unknown)
    lua_pushstring(L, name.c_str()); // 6: name
    lua_pushstring(L, "");         // 7: realm
    return 7;
}


static int lua_UnitClassification(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { lua_pushstring(L, "normal"); return 1; }
    const char* uid = luaL_optstring(L, 1, "target");
    std::string uidStr(uid);
    toLowerInPlace(uidStr);
    uint64_t guid = resolveUnitGuid(gh, uidStr);
    if (guid == 0) { lua_pushstring(L, "normal"); return 1; }
    auto entity = gh->getEntityManager().getEntity(guid);
    if (!entity || entity->getType() == game::ObjectType::PLAYER) {
        lua_pushstring(L, "normal");
        return 1;
    }
    auto unit = std::dynamic_pointer_cast<game::Unit>(entity);
    if (!unit) { lua_pushstring(L, "normal"); return 1; }
    int rank = gh->getCreatureRank(unit->getEntry());
    switch (rank) {
        case 1:  lua_pushstring(L, "elite"); break;
        case 2:  lua_pushstring(L, "rareelite"); break;
        case 3:  lua_pushstring(L, "worldboss"); break;
        case 4:  lua_pushstring(L, "rare"); break;
        default: lua_pushstring(L, "normal"); break;
    }
    return 1;
}

// GetComboPoints("player"|"vehicle", "target") → number
static int lua_GetComboPoints(lua_State* L) {
    auto* gh = getGameHandler(L);
    lua_pushnumber(L, gh ? gh->getComboPoints() : 0);
    return 1;
}

// UnitReaction(unit, otherUnit) → 1-8 (hostile to exalted)
// Simplified: hostile=2, neutral=4, friendly=5
static int lua_UnitReaction(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { return luaReturnNil(L); }
    const char* uid1 = luaL_checkstring(L, 1);
    const char* uid2 = luaL_checkstring(L, 2);
    auto* unit2 = resolveUnit(L, uid2);
    if (!unit2) { return luaReturnNil(L); }
    // If unit2 is the player, always friendly to self
    std::string u1(uid1);
    toLowerInPlace(u1);
    std::string u2(uid2);
    toLowerInPlace(u2);
    uint64_t g1 = resolveUnitGuid(gh, u1);
    uint64_t g2 = resolveUnitGuid(gh, u2);
    if (g1 == g2) { lua_pushnumber(L, 5); return 1; } // same unit = friendly
    if (unit2->isHostile()) {
        lua_pushnumber(L, 2); // hostile
    } else {
        lua_pushnumber(L, 5); // friendly
    }
    return 1;
}

/// Which side the battlefield scoreboard is showing: 1 Alliance, 0 Horde,
/// -1 both. Set by the frame's tabs through SetBattlefieldScoreFaction.
static int& battlefieldScoreFaction() {
    static int faction = -1;
    return faction;
}

/// Which column the scoreboard is sorted by, and which way.
///
/// The direction lives here because the interface does not keep it: clicking a
/// column hands the same name over again and expects the order to reverse,
/// which only works if something remembers what it was.
static std::string& battlefieldSortColumn() {
    static std::string column;
    return column;
}
static bool& battlefieldSortDescending() {
    static bool descending = true;
    return descending;
}

/// The scoreboard rows that pass the current filter, in the current order.
///
/// Pointers into the scoreboard rather than copies: this is rebuilt on every
/// GetBattlefieldScore call, once per row per redraw, and a battleground holds
/// forty players.
static std::vector<const game::BgPlayerScore*>
filteredBgScores(const game::BgScoreboardData& sb) {
    std::vector<const game::BgPlayerScore*> rows;
    rows.reserve(sb.players.size());
    const int want = battlefieldScoreFaction();
    for (const auto& p : sb.players) {
        // A row with no team on the wire cannot be filtered out of a faction
        // tab - a battleground sends none, so every row shows on both tabs
        // rather than all of them landing on Horde because zero is Horde.
        if (want < 0 || !p.hasTeam || static_cast<int>(p.team) == want)
            rows.push_back(&p);
    }

    const std::string& column = battlefieldSortColumn();
    if (column.empty()) return rows;

    // Only the columns this client has the numbers for. "class" is not one of
    // them - the scoreboard packet carries no class.
    //
    // Damage and healing were excluded here for the same reason, back when the
    // parser skipped their eight bytes and every row read zero; sorting by
    // them would have rearranged nothing while looking like it worked. The
    // parser reads them now and GetBattlefieldScore hands them to the frame,
    // so the two columns show real numbers - and were the only two whose
    // header did nothing when clicked.
    auto key = [&column](const game::BgPlayerScore* p) -> double {
        if (column == "kills")   return p->killingBlows;
        if (column == "deaths")  return p->deaths;
        if (column == "hk")      return p->honorableKills;
        if (column == "cp")      return p->bonusHonor;
        if (column == "team")    return p->team;
        if (column == "damage")  return p->damageDone;
        if (column == "healing") return p->healingDone;
        return 0.0;
    };
    const bool byName = (column == "name");
    const bool descending = battlefieldSortDescending();
    std::stable_sort(rows.begin(), rows.end(),
                     [&](const game::BgPlayerScore* a, const game::BgPlayerScore* b) {
                         if (byName) {
                             return descending ? a->name > b->name : a->name < b->name;
                         }
                         return descending ? key(a) > key(b) : key(a) < key(b);
                     });
    return rows;
}

// UnitIsConnected(unit) → boolean
static int lua_UnitIsConnected(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { return luaReturnFalse(L); }
    const char* uid = luaL_optstring(L, 1, "player");
    std::string uidStr(uid);
    toLowerInPlace(uidStr);
    uint64_t guid = resolveUnitGuid(gh, uidStr);
    if (guid == 0) { return luaReturnFalse(L); }
    // Player is always connected
    if (guid == gh->getPlayerGuid()) { lua_pushboolean(L, 1); return 1; }
    // Check party/raid member online status
    const auto& pd = gh->getPartyData();
    for (const auto& m : pd.members) {
        if (m.guid == guid) {
            lua_pushboolean(L, m.isOnline ? 1 : 0);
            return 1;
        }
    }
    // Non-party entities that exist are considered connected
    auto entity = gh->getEntityManager().getEntity(guid);
    lua_pushboolean(L, entity ? 1 : 0);
    return 1;
}

// HasAction(slot) → boolean (1-indexed slot)

void registerUnitLuaAPI(lua_State* L) {
    static const struct { const char* name; lua_CFunction func; } api[] = {
                {"UnitName",      lua_UnitName},
                {"UnitFullName",  lua_UnitName},
                {"GetUnitName",   lua_UnitName},
                {"UnitHealth",    lua_UnitHealth},
                {"UnitHealthMax", lua_UnitHealthMax},
                {"UnitPower",     lua_UnitPower},
                {"UnitPowerMax",  lua_UnitPowerMax},
                {"UnitMana",      lua_UnitPower},
                {"UnitManaMax",   lua_UnitPowerMax},
                {"UnitRage",      lua_UnitPower},
                {"UnitEnergy",    lua_UnitPower},
                {"UnitFocus",     lua_UnitPower},
                {"UnitRunicPower", lua_UnitPower},
                {"UnitLevel",     lua_UnitLevel},
                {"UnitExists",    lua_UnitExists},
                {"UnitIsDead",    lua_UnitIsDead},
                // UnitUsingVehicle(unit) - in a vehicle, or moving between its
                // seats. This client models the vehicle a player is in and not
                // its seats, so the second half never happens and the first is
                // exactly isInVehicle.
                //
                // Only the player: a vehicle another unit is riding is not
                // something this client is told about, and answering false for
                // one is the truth as far as it knows rather than a guess.
                {"UnitUsingVehicle", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            std::string uid(luaL_optstring(L, 1, "player"));
            toLowerInPlace(uid);
            lua_pushboolean(L, (gh && uid == "player" && gh->isInVehicle()) ? 1 : 0);
            return 1;
        }},
                // UnitVehicleSeatInfo(unit, seat) → controlType, occupantName,
                // serverName, ejectable, canSwitchSeats.
                //
                // Seats are not modelled: SMSG_PLAYER_VEHICLE_DATA carries the
                // vehicle a player is in and nothing about who else is aboard.
                // Nothing rather than an invented seat - the caller shows an
                // occupant's name from it, and a made-up one would be a name.
                {"UnitVehicleSeatInfo", [](lua_State* L) -> int {
            for (int i = 0; i < 5; ++i) lua_pushnil(L);
            return 5;
        }},
                {"UnitIsGhost",   lua_UnitIsGhost},
                {"UnitIsDeadOrGhost", lua_UnitIsDeadOrGhost},
                {"UnitIsAFK",     lua_UnitIsAFK},
                {"UnitIsDND",     lua_UnitIsDND},
                {"UnitPlayerControlled", lua_UnitPlayerControlled},
                {"UnitIsTapped",        lua_UnitIsTapped},
                {"UnitIsTappedByPlayer", lua_UnitIsTappedByPlayer},
                {"UnitIsTappedByAllThreatList", lua_UnitIsTappedByAllThreatList},
                {"UnitIsVisible",       lua_UnitIsVisible},
                {"UnitGroupRolesAssigned", lua_UnitGroupRolesAssigned},
                {"UnitCanAttack",       lua_UnitCanAttack},
                {"UnitCanCooperate",    lua_UnitCanCooperate},
                {"UnitCreatureFamily",  lua_UnitCreatureFamily},
                {"UnitOnTaxi",          lua_UnitOnTaxi},
                {"UnitThreatSituation", lua_UnitThreatSituation},
                {"UnitDetailedThreatSituation", lua_UnitDetailedThreatSituation},
                {"UnitSex",       lua_UnitSex},
                {"UnitClass",     lua_UnitClass},
                {"UnitArmor",     [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            // Whose armour. PaperDollFrame_SetArmor is shared between the two
            // sheets and the pet tab calls it with "Pet" - capitalised, which
            // is why this compares lowered.
            std::string who(luaL_optstring(L, 1, "player"));
            toLowerInPlace(who);
            int32_t armor = 0;
            if (gh) {
                armor = (who == "pet") ? gh->getPetResistances()[0]
                      : (who == "player") ? gh->getArmorRating() : 0;
            }
            if (armor < 0) armor = 0;
            lua_pushnumber(L, armor); // base
            lua_pushnumber(L, armor); // effective
            lua_pushnumber(L, armor); // armor (again for compat)
            lua_pushnumber(L, 0);     // posBuff
            lua_pushnumber(L, 0);     // negBuff
            return 5;
        }},
                {"UnitResistance", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            int school = static_cast<int>(luaL_optnumber(L, 2, 0));
            int32_t val = 0;
            // Whose resistance, for the same reason as UnitStat above.
            std::string who(luaL_optstring(L, 1, "player"));
            toLowerInPlace(who);
            if (gh && school >= 0 && school <= 6) {
                if (who == "pet") {
                    val = gh->getPetResistances()[static_cast<size_t>(school)];
                } else if (who == "player") {
                    // Physical is armor, and it is index zero of the same block.
                    val = (school == 0) ? gh->getArmorRating() : gh->getResistance(school);
                }
            }
            if (val < 0) val = 0;
            lua_pushnumber(L, val); // base
            lua_pushnumber(L, val); // effective
            lua_pushnumber(L, 0);   // posBuff
            lua_pushnumber(L, 0);   // negBuff
            return 4;
        }},
                {"UnitStat",      lua_UnitStat},
                {"GetDodgeChance",    lua_GetDodgeChance},
                {"GetParryChance",    lua_GetParryChance},
                {"GetBlockChance",    lua_GetBlockChance},
                {"GetCritChance",     lua_GetCritChance},
                {"GetRangedCritChance", lua_GetRangedCritChance},
                {"GetSpellCritChance",  lua_GetSpellCritChance},
                {"GetCombatRating",     lua_GetCombatRating},
                {"GetSpellBonusDamage", lua_GetSpellBonusDamage},
                {"GetSpellBonusHealing", lua_GetSpellBonusHealing},
                {"GetAttackPowerForStat", lua_GetAttackPower},
                {"GetRangedAttackPower",  lua_GetRangedAttackPower},
                {"IsInGroup",     lua_IsInGroup},
                {"IsInRaid",      lua_IsInRaid},
                {"GetShapeshiftFormInfo", [](lua_State* L) -> int {
            // GetShapeshiftFormInfo(index) → icon, name, isActive, isCastable
            auto* gh = getGameHandler(L);
            int index = static_cast<int>(luaL_checknumber(L, 1));
            if (!gh || index < 1) { return luaReturnNil(L); }
            uint8_t classId = gh->getPlayerClass();
            uint8_t currentForm = gh->getShapeshiftFormId();

            // The one table CastShapeshiftForm and GetNumShapeshiftForms
            // also read, filtered the same way. The bar walks
            // 1..GetNumShapeshiftForms() and asks about each index, so all
            // three have to be looking at the same list in the same order.
            const auto forms = game::knownShapeshiftForms(classId, gh->getKnownSpells());
            if (static_cast<size_t>(index) > forms.size()) { return luaReturnNil(L); }
            const auto& fi = forms[static_cast<size_t>(index) - 1];
            lua_pushstring(L, fi.icon);                          // icon
            lua_pushstring(L, fi.name);                          // name
            lua_pushboolean(L, currentForm == fi.formId ? 1 : 0); // isActive
            lua_pushboolean(L, 1);                               // isCastable
            return 4;
        }},
                {"UnitIsPVP", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* uid = luaL_optstring(L, 1, "player");
            if (!gh) { return luaReturnFalse(L); }
            uint64_t guid = resolveUnitGuid(gh, std::string(uid));
            if (guid == 0) { return luaReturnFalse(L); }
            auto entity = gh->getEntityManager().getEntity(guid);
            if (!entity) { return luaReturnFalse(L); }
            // UNIT_FLAG_PVP = 0x00001000
            uint32_t flags = entity->getField(game::fieldIndex(game::UF::UNIT_FIELD_FLAGS));
            lua_pushboolean(L, (flags & 0x00001000) ? 1 : 0);
            return 1;
        }},
                {"UnitIsPVPFreeForAll", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* uid = luaL_optstring(L, 1, "player");
            if (!gh) { return luaReturnFalse(L); }
            uint64_t guid = resolveUnitGuid(gh, std::string(uid));
            if (guid == 0) { return luaReturnFalse(L); }
            auto entity = gh->getEntityManager().getEntity(guid);
            if (!entity) { return luaReturnFalse(L); }
            // FFA PvP is PLAYER_FLAGS bit 0x80, NOT UNIT_FIELD_FLAGS bit 0x00080000
            // (which is UNIT_FLAG_PACIFIED - would flag pacified mobs as FFA-PVP).
            uint32_t pf = entity->getField(game::fieldIndex(game::UF::PLAYER_FLAGS));
            lua_pushboolean(L, (pf & 0x00000080) ? 1 : 0);
            return 1;
        }},
                // The honour panel prints "(Rank N)" from the second return
                // and concatenates it unguarded, so a missing one takes the
                // panel's whole update with it. Rank zero is what an
                // unranked character has.
                // Whether the off hand holds a weapon rather than a shield or
                // a held item. DurabilityFrame_SetAlerts branches on it to
                // decide which of its two off-hand icons to show.
                {"OffhandHasWeapon", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) { lua_pushboolean(L, 0); return 1; }
            const auto& slot = gh->getInventory().getEquipSlot(game::EquipSlot::OFF_HAND);
            if (slot.empty()) { lua_pushboolean(L, 0); return 1; }
            const auto* info = gh->getItemInfo(slot.item.itemId);
            const uint8_t t = info ? info->inventoryType : 0;
            // A weapon, not a shield or a held book: OFF_HAND in this table
            // is the held-in-off-hand class, which is not one.
            lua_pushboolean(L, (t == game::InvType::ONE_HAND ||
                                t == game::InvType::TWO_HAND) ? 1 : 0);
            return 1;
        }},
                // The colour a unit's name is drawn in: red for hostile, green
                // for friendly, yellow for neutral. Unit frames read all four
                // components straight into SetTextColor.
                {"UnitSelectionColor", [](lua_State* L) -> int {
            const char* uid = luaL_optstring(L, 1, "player");
            auto* unit = resolveUnit(L, uid);
            float r = 0.0f, g = 1.0f, b = 0.0f;
            if (unit && unit->isHostile()) { r = 1.0f; g = 0.0f; b = 0.0f; }
            lua_pushnumber(L, r);
            lua_pushnumber(L, g);
            lua_pushnumber(L, b);
            lua_pushnumber(L, 1.0);
            return 4;
        }},
                {"UnitIsPartyLeader", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* uid = luaL_optstring(L, 1, "player");
            std::string uidStr(uid);
            toLowerInPlace(uidStr);
            const uint64_t guid = gh ? resolveUnitGuid(gh, uidStr) : 0;
            lua_pushboolean(L, (gh && guid != 0 &&
                                gh->getPartyData().leaderGuid == guid) ? 1 : 0);
            return 1;
        }},
                {"UnitIsCorpse", [](lua_State* L) -> int {
            const char* uid = luaL_optstring(L, 1, "player");
            auto* unit = resolveUnit(L, uid);
            lua_pushboolean(L, (unit && unit->getHealth() == 0) ? 1 : 0);
            return 1;
        }},
                // Whether the first unit may help the second - true between
                // anything not hostile to each other.
                {"UnitCanAssist", [](lua_State* L) -> int {
            auto* other = resolveUnit(L, luaL_optstring(L, 2, "target"));
            lua_pushboolean(L, (other && !other->isHostile()) ? 1 : 0);
            return 1;
        }},
                {"GetPVPRankInfo", [](lua_State* L) -> int {
            const int rank = static_cast<int>(luaL_optnumber(L, 1, 0));
            lua_pushstring(L, "");          // rank name
            lua_pushnumber(L, rank > 0 ? rank : 0);
            return 2;
        }},
                // Honour and arena totals, in the shapes their panels expect.
                // Zeroes rather than nothing: every one of these is read
                // straight into arithmetic or a format string.
                {"GetPVPLifetimeStats", [](lua_State* L) -> int {
            lua_pushnumber(L, 0); lua_pushnumber(L, 0); lua_pushnumber(L, 0);
            return 3;
        }},
                {"GetPVPSessionStats", [](lua_State* L) -> int {
            lua_pushnumber(L, 0); lua_pushnumber(L, 0);
            return 2;
        }},
                {"GetPVPYesterdayStats", [](lua_State* L) -> int {
            lua_pushnumber(L, 0); lua_pushnumber(L, 0);
            return 2;
        }},
                // Seven returns, as the battleground panel reads them:
                // status, map, instance, level range, team size, registered.
                // Stopping at three left teamSize nil and the panel formatted
                // a number from nothing.
                // GetBattlefieldStatus(queue) → status, map, instance, the level
                // range, the arena team size and whether it is a rated match.
                //
                // "none" for every queue, while SMSG_BATTLEFIELD_STATUS has been
                // parsed into three slots carrying exactly this. So the PVP
                // frame showed nothing queued however long the player had been
                // waiting, and battlefieldframe.lua's own test -
                // `queueStatus ~= "none"` - never passed.
                //
                // The status tokens are the four the interface compares against
                // and the ids are AzerothCore's BattlegroundStatus: 1 waiting in
                // the queue, 2 invited and waiting to accept, 3 in progress, 4
                // ending. Four reads as active because the battleground is
                // still the one the player is standing in.
                {"GetBattlefieldStatus", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int idx = static_cast<int>(luaL_optnumber(L, 1, 0));
            const char* status = "none";
            std::string mapName;
            uint32_t minLevel = 0, maxLevel = 0, instanceId = 0;
            uint8_t teamSize = 0;
            bool isRated = false;
            if (gh && idx >= 1 && idx <= 3) {
                const auto& q = gh->getBgQueues()[static_cast<size_t>(idx) - 1];
                switch (q.statusId) {
                    case 1: status = "queued";  break;
                    case 2: status = "confirm"; break;
                    case 3: case 4: status = "active"; break;
                    default: break;
                }
                if (q.statusId != 0) {
                    mapName = q.bgName;
                    teamSize = q.arenaType;
                    instanceId = q.instanceId;
                    isRated = q.isRated;
                    // The range arrives with the status itself. It used to be
                    // looked up in the available-battleground list, which only
                    // turns up at a battlemaster - so a queue joined anywhere
                    // else answered a range of zero to zero for as long as it
                    // lasted. The list is still the fallback, for a queue
                    // carried over from a login where the status came first.
                    minLevel = q.minLevel;
                    maxLevel = q.maxLevel;
                    if (minLevel == 0 && maxLevel == 0) {
                        for (const auto& bg : gh->getAvailableBgs()) {
                            if (bg.bgTypeId != q.bgTypeId) continue;
                            minLevel = bg.minLevel;
                            maxLevel = bg.maxLevel;
                            break;
                        }
                    }
                }
            }
            // An empty queue answers a nil map name, which is what the real
            // client does and what battlefieldframe.lua's `if ( mapName )` is
            // written for. Its own path rather than a conditional push, so the
            // seven values read in order on both - the return-order sweep reads
            // the sequence of pushes, and a branch in the middle of one is a
            // sequence it cannot follow.
            if (mapName.empty()) {
                lua_pushstring(L, status);
                lua_pushnil(L);
                lua_pushnumber(L, 0);
                lua_pushnumber(L, 0);
                lua_pushnumber(L, 0);
                lua_pushnumber(L, 0);
                lua_pushboolean(L, 0);
                return 7;
            }
            lua_pushstring(L, status);
            lua_pushstring(L, mapName.c_str());
            // Both were answered as a flat zero with a comment saying the wire
            // does not carry them. It carries both - they sit between the map
            // type and the status, in the stretch this used to read two bytes
            // short of. The interface uses them: battlefieldframe.lua appends a
            // non-zero instance to the name, which is what makes a queue read
            // "Warsong Gulch 2".
            lua_pushnumber(L, instanceId);
            lua_pushnumber(L, minLevel);
            lua_pushnumber(L, maxLevel);
            lua_pushnumber(L, teamSize);
            // A boolean, not a number. The real binding answers true or false
            // and the interface asks `if ( registeredMatch )` - and zero is
            // true in Lua, so answering 0 for "not rated" labelled every arena
            // queue a rated match.
            lua_pushboolean(L, isRated ? 1 : 0);
            return 7;
        }},
                // The money a quest asks for, which the tracker compares
                // against the player's before calling an objective complete.
                // SetBattlefieldScoreFaction(faction) - show one side only.
                //
                // The scoreboard's tabs are a filter, not a sort: 1 is
                // Alliance, 0 is Horde, and nil is both. It has to reach the
                // two functions below, because the rows the frame draws are
                // the rows they hand back - the frame does no filtering of its
                // own and would otherwise show everyone under either tab.
                {"SetBattlefieldScoreFaction", [](lua_State* L) -> int {
            if (lua_isnoneornil(L, 1)) battlefieldScoreFaction() = -1;
            else battlefieldScoreFaction() = static_cast<int>(lua_tonumber(L, 1));
            return 0;
        }},
                // SortBattlefieldScoreData(column) - order the rows.
                //
                // The same column twice reverses, which is the real client's
                // behaviour and has to be remembered here: the interface hands
                // over only the column name and keeps no direction of its own.
                //
                // A name is sensible ascending and a score is not, so a fresh
                // column starts descending for everything but the name.
                {"SortBattlefieldScoreData", [](lua_State* L) -> int {
            std::string column(luaL_optstring(L, 1, ""));
            if (column.empty()) return 0;
            if (column == battlefieldSortColumn()) {
                battlefieldSortDescending() = !battlefieldSortDescending();
            } else {
                battlefieldSortColumn() = column;
                battlefieldSortDescending() = (column != "name");
            }
            return 0;
        }},
                {"GetNumBattlefieldScores", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const auto* sb = gh ? gh->getBgScoreboard() : nullptr;
            lua_pushnumber(L, sb ? static_cast<double>(filteredBgScores(*sb).size()) : 0.0);
            return 1;
        }},
                {"GetBattlefieldScore", [](lua_State* L) -> int {
            // GetBattlefieldScore(index) → name, killingBlows, honorableKills, deaths, honorGained, faction, rank, race, class, classToken, damageDone, healingDone
            auto* gh = getGameHandler(L);
            int index = static_cast<int>(luaL_checknumber(L, 1));
            const auto* sb = gh ? gh->getBgScoreboard() : nullptr;
            if (!sb) return luaReturnNil(L);
            const auto rows = filteredBgScores(*sb);
            if (index < 1 || index > static_cast<int>(rows.size())) {
                return luaReturnNil(L);
            }
            const auto& p = *rows[static_cast<size_t>(index - 1)];
            lua_pushstring(L, p.name.c_str());     // name
            lua_pushnumber(L, p.killingBlows);      // killingBlows
            lua_pushnumber(L, p.honorableKills);    // honorableKills
            lua_pushnumber(L, p.deaths);            // deaths
            lua_pushnumber(L, p.bonusHonor);        // honorGained
            lua_pushnumber(L, p.team);              // faction (0=Horde,1=Alliance)
            lua_pushnumber(L, 0);                   // rank
            lua_pushstring(L, "");                  // race
            lua_pushstring(L, "");                  // class
            // Nil, not a class. The packet carries none - AppendToPacket sends
            // a guid and six numbers - and the scoreboard asks for exactly
            // this case: `if (classToken) then ... else buttonClass:Hide()`.
            // So absence is an answer it handles, and "WARRIOR" was not a
            // safer default than nil but a wrong one that looks like data: it
            // drew a warrior's icon beside all forty players in the
            // battleground, every one of them.
            lua_pushnil(L);                         // classToken
            lua_pushnumber(L, p.damageDone);        // damageDone
            lua_pushnumber(L, p.healingDone);       // healingDone
            return 12;
        }},
                // ---- The scoreboard's per-battleground columns ----
                //
                // Every battleground adds its own: flags captured in Warsong,
                // bases assaulted in Arathi, towers defended in Alterac. The
                // server names each one and sends a value per player, and this
                // client has parsed both into BgPlayerScore::bgStats all along
                // - nothing read them, and GetNumBattlefieldStats answered zero
                // from the counting stub, so the scoreboard drew the common
                // columns and stopped.
                {"GetNumBattlefieldStats", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const auto* sb = gh ? gh->getBgScoreboard() : nullptr;
            size_t columns = 0;
            // The widest row rather than the first: a row that arrived short
            // would otherwise decide the column count for the whole table.
            if (sb) for (const auto& p : sb->players) columns = std::max(columns, p.bgStats.size());
            lua_pushnumber(L, static_cast<double>(columns));
            return 1;
        }},
                // GetBattlefieldStatInfo(index) → text, icon, tooltip
                //
                // The icon is "" rather than nil on purpose: the frame tests it
                // with ~= "" and then concatenates the faction onto it, so nil
                // passes the test and dies on the concatenation. This client
                // has no artwork per column, and "" is how the frame is told so.
                {"GetBattlefieldStatInfo", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int index = static_cast<int>(luaL_checknumber(L, 1));
            const auto* sb = gh ? gh->getBgScoreboard() : nullptr;
            if (!sb || index < 1) return luaReturnNil(L);
            // The label is this client's to supply. BuildObjectivesBlock
            // sends a count and that many bare numbers, so the name this used
            // to read out of the row was never on the wire - it was bytes of
            // the value before it, read as a string.
            //
            // Which column is which follows from the map: Warsong sends flags
            // captured and returned, Alterac Valley five graveyard and tower
            // counts, Eye of the Storm one. The table is beside the world-state
            // one it belongs with.
            const char* label = game::bgObjectiveLabel(
                gh->getCurrentMapId(), static_cast<size_t>(index - 1));
            if (!label) return luaReturnNil(L);
            lua_pushstring(L, label);
            lua_pushstring(L, "");
            lua_pushstring(L, label);
            return 3;
        }},
                // GetBattlefieldStatData(playerIndex, statIndex) → the value.
                //
                // Always a number. The frame compares it against zero the line
                // after it asks, so nil is not "no value" there, it is an error.
                {"GetBattlefieldStatData", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int playerIndex = static_cast<int>(luaL_checknumber(L, 1));
            const int statIndex   = static_cast<int>(luaL_checknumber(L, 2));
            const auto* sb = gh ? gh->getBgScoreboard() : nullptr;
            double value = 0.0;
            if (sb && playerIndex >= 1 && playerIndex <= static_cast<int>(sb->players.size())) {
                const auto& stats = sb->players[static_cast<size_t>(playerIndex - 1)].bgStats;
                if (statIndex >= 1 && statIndex <= static_cast<int>(stats.size()))
                    value = static_cast<double>(stats[static_cast<size_t>(statIndex - 1)].second);
            }
            lua_pushnumber(L, value);
            return 1;
        }},
                // GetBattlefieldTeamInfo(faction) → name, rating, newRating, skill
                //
                // Arena only. The frame subtracts the two ratings to show the
                // change, so the old one is derived from the new one and the
                // change the server sent - which makes the subtraction come out
                // as that change whichever way round the server means it.
                //
                // The matchmaker rating is not parsed, and nil is the right
                // answer for it: the frame prints "-------" for a rating it was
                // not given, where a zero would read as a real one.
                {"GetBattlefieldTeamInfo", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int faction = static_cast<int>(luaL_checknumber(L, 1));
            const auto* sb = gh ? gh->getBgScoreboard() : nullptr;
            if (!sb || !sb->isArena || faction < 0 || faction > 1) return luaReturnNil(L);
            const auto& team = sb->arenaTeams[faction];
            lua_pushstring(L, team.teamName.c_str());
            lua_pushnumber(L, static_cast<double>(team.newRating) -
                              static_cast<double>(team.ratingChange));
            lua_pushnumber(L, static_cast<double>(team.newRating));
            lua_pushnil(L);
            return 4;
        }},
                // ReportPlayerIsPVPAFK(name) - flag someone as not taking part.
                //
                // Named rather than targeted: the scoreboard row knows who it
                // is showing but not their guid, so the name is matched against
                // the scoreboard this client already holds. A name that is not
                // on it cannot be reported, which is also true in the real
                // client - the report is only offered from a row.
                {"ReportPlayerIsPVPAFK", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* name = lua_isstring(L, 1) ? lua_tostring(L, 1) : nullptr;
            const auto* sb = gh ? gh->getBgScoreboard() : nullptr;
            if (!gh || !sb || !name || !*name) return 0;
            for (const auto& p : sb->players) {
                if (p.name == name) { gh->reportPvpAfk(p.guid); return 0; }
            }
            return 0;
        }},
                {"GetBattlefieldWinner", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const auto* sb = gh ? gh->getBgScoreboard() : nullptr;
            if (sb && sb->hasWinner) lua_pushnumber(L, sb->winner);
            else lua_pushnil(L);
            return 1;
        }},
                {"RequestBattlefieldScoreData", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (gh) gh->requestPvpLog();
            return 0;
        }},
                {"AcceptBattlefieldPort", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            int accept = lua_toboolean(L, 2);
            if (gh) {
                if (accept) gh->acceptBattlefield();
                else gh->declineBattlefield();
            }
            return 0;
        }},
                {"UnitRace",          lua_UnitRace},
                {"UnitPowerType",     lua_UnitPowerType},
                {"GetNumGroupMembers", lua_GetNumGroupMembers},
                {"UnitGUID",          lua_UnitGUID},
                {"UnitIsPlayer",      lua_UnitIsPlayer},
                {"UnitPVPName",       lua_UnitPVPName},
                {"InCombatLockdown",  lua_InCombatLockdown},
                {"UnitDistanceSquared", lua_UnitDistanceSquared},
                {"CheckInteractDistance", lua_CheckInteractDistance},
                {"UnitInRange",           lua_UnitInRange},
                {"IsRaidLeader",          lua_IsRaidLeader},
                // The class token without the localised name beside it, which
                // is all UnitClassBase is: the same second return UnitClass
                // already computes.
                {"UnitClassBase",         [](lua_State* L) -> int {
            lua_getglobal(L, "UnitClass");
            lua_pushvalue(L, 1);
            if (lua_pcall(L, 1, 3, 0) != 0) { lua_pop(L, 1); lua_pushnil(L); return 1; }
            // UnitClass answers name, token, id; the token is what is wanted.
            lua_remove(L, -3);
            lua_pop(L, 1);
            return 1;
        }},
                {"IsMounted",         lua_IsMounted},
                {"IsFlying",          lua_IsFlying},
                {"IsSwimming",        lua_IsSwimming},
                {"IsResting",         lua_IsResting},
                {"IsFalling",         lua_IsFalling},
                {"IsStealthed",       lua_IsStealthed},
                {"GetUnitSpeed",      lua_GetUnitSpeed},
                {"UnitAffectingCombat", lua_UnitAffectingCombat},
                {"GetNumRaidMembers",   lua_GetNumRaidMembers},
                {"GetNumPartyMembers",  lua_GetNumPartyMembers},
                // The counts before the dungeon finder inflates them. There is
                // no dungeon finder here, so they are the same number - and
                // UIParent does arithmetic on them, where absent is an error
                // rather than a zero.
                {"GetRealNumRaidMembers",  lua_GetNumRaidMembers},
                {"GetRealNumPartyMembers", lua_GetNumPartyMembers},
                {"HasFullControl",      lua_HasFullControl},
                {"GetPetSpellBonusDamage", lua_GetPetSpellBonusDamage},
                {"RetrieveCorpse",      lua_RetrieveCorpse},
                {"Dismount",            lua_Dismount},
                {"StartAttack",         lua_StartAttack},
                {"StopAttack",          lua_StopAttack},
                {"UnitInParty",         lua_UnitInParty},
                {"UnitInRaid",          lua_UnitInRaid},
                {"UnitHasVehicleUI",    lua_UnitHasVehicleUI},
                // Which vehicle art a unit's frame should wear. Reached only
                // behind UnitHasVehicleUI, which answers false here because no
                // vehicle is modelled - so this is unreachable today and nil
                // is the honest answer for a unit that is not in one. It is
                // bound because leaving it out is the difference between the
                // party frames calling nothing this client lacks and calling
                // one thing it does.
                {"UnitVehicleSkin",     [](lua_State* L) -> int { return luaReturnNil(L); }},
                {"UnitInVehicle",       lua_UnitInVehicle},
                {"UnitControllingVehicle", lua_UnitControllingVehicle},
                {"UnitIsPossessed",     lua_UnitIsPossessed},
                {"UnitIsCharmed",       lua_UnitIsCharmed},
                {"UnitIsTalking",       lua_UnitIsTalking},
                {"UnitInBattleground",  lua_UnitInBattleground},
                {"UnitPlayerOrPetInParty", lua_UnitPlayerOrPetInParty},
                {"UnitPlayerOrPetInRaid",  lua_UnitPlayerOrPetInRaid},
                {"UnitCharacterPoints", lua_UnitCharacterPoints},
                {"PetHasActionBar",     lua_PetHasActionBar},
                {"PetCanBeDismissed",   lua_PetCanBeDismissed},
                {"PetCanBeAbandoned",   lua_PetCanBeAbandoned},
                {"GetRaidRosterInfo",   lua_GetRaidRosterInfo},
                {"GetThreatStatusColor", lua_GetThreatStatusColor},
                {"GetReadyCheckStatus", lua_GetReadyCheckStatus},
                {"RegisterUnitWatch",   lua_RegisterUnitWatch},
                {"UnregisterUnitWatch", lua_UnregisterUnitWatch},
                {"UnitIsUnit",          lua_UnitIsUnit},
                {"UnitIsFriend",        lua_UnitIsFriend},
                {"UnitIsEnemy",         lua_UnitIsEnemy},
                {"UnitCreatureType",    lua_UnitCreatureType},
                {"UnitClassification",  lua_UnitClassification},
                {"UnitReaction",        lua_UnitReaction},
                {"UnitIsConnected",     lua_UnitIsConnected},
                {"GetComboPoints",      lua_GetComboPoints},
                {"GetPlayerInfoByGUID",  lua_GetPlayerInfoByGUID},
                {"UnitXP",                  lua_UnitXP},
                {"UnitXPMax",               lua_UnitXPMax},
                {"GetXPExhaustion",         lua_GetXPExhaustion},
                {"GetTimeToWellRested",     lua_GetTimeToWellRested},
                {"UnitAttackPower",         lua_UnitAttackPower},
                {"UnitRangedAttackPower",   lua_UnitRangedAttackPower},
                {"UnitDefense",             lua_UnitDefense},
                {"UnitAttackSpeed",         lua_UnitAttackSpeed},
                {"UnitDamage",              lua_UnitDamage},
                {"UnitRangedDamage",        lua_UnitRangedDamage},
                {"UnitRangedAttack",        lua_UnitRangedAttack},
                {"UnitAttackBothHands",     lua_UnitAttackBothHands},
                {"GetManaRegen",            lua_GetManaRegen},
                {"GetMaxCombatRatingBonus", lua_GetMaxCombatRatingBonus},
                {"GetUnitMaxHealthModifier", lua_GetUnitMaxHealthModifier},
                {"GetInventoryItemCooldown", lua_GetInventoryItemCooldown},
                {"SetPortraitTexture",         lua_SetPortraitTexture},
                {"GetHairCustomization",       lua_GetHairCustomization},
                {"GetFacialHairCustomization", lua_GetFacialHairCustomization},
                {"CanAlterSkin",               lua_CanAlterSkin},
                {"GetBarberShopStyleInfo",     lua_GetBarberShopStyleInfo},
                {"SetNextBarberShopStyle",     lua_SetNextBarberShopStyle},
                {"GetBarberShopTotalCost",     lua_GetBarberShopTotalCost},
                {"BarberShopReset",            lua_BarberShopReset},
                {"CancelBarberShop",           lua_CancelBarberShop},
                // The Okay button, named as an OnClick attribute rather than
                // called from a script body - which is exactly why the
                // readiness report called this element finished while its one
                // committing action raised.
                {"ApplyBarberShopStyle", [](lua_State* L) -> int {
            if (auto* svc = getLuaServices(L); svc && svc->barberApply) svc->barberApply();
            return 0;
        }},
                // HasWandEquipped() - a wand in the ranged slot, which the
                // character sheet needs because a wand's damage is read
                // differently from a bow's: PaperDollFrame_SetRangedDamage
                // takes the plain average and skips the attack-power bonus a
                // physical ranged weapon gets. Answering no meant a wand user's
                // ranged damage was computed as though the wand were a bow.
                //
                // Weapon class 2, subclass 19, off the same table the client's
                // own subclass names come from.
                {"HasWandEquipped",         [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) { lua_pushboolean(L, 0); return 1; }
            const auto& sl = gh->getInventory().getEquipSlot(game::EquipSlot::RANGED);
            bool wand = false;
            if (!sl.empty()) {
                if (const auto* info = gh->getItemInfo(sl.item.itemId); info && info->valid) {
                    wand = (info->itemClass == 2 && info->subClass == 19);
                }
            }
            lua_pushboolean(L, wand);
            return 1;
        }},
                // UnitHasRelicSlot(unit) - the four classes whose ranged slot
                // holds a relic instead of a weapon: paladin librams, death
                // knight sigils, shaman totems and druid idols. Answering no
                // for all of them made the paperdoll label that slot "Ranged"
                // and read its stats as a ranged weapon's.
                {"UnitHasRelicSlot",        [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* uid = luaL_optstring(L, 1, "player");
            // Only the player's class is known well enough to answer this.
            if (!gh || std::string(uid) != "player") { lua_pushboolean(L, 0); return 1; }
            const uint8_t c = gh->getPlayerClass();
            lua_pushboolean(L, c == 2 || c == 6 || c == 7 || c == 11);
            return 1;
        }},
                // Whether the repair cursor is up. A flat false meant the
                // merchant's repair button re-armed on every click instead of
                // toggling, and neither the bag nor the paperdoll tooltip ever
                // showed an item's repair cost.
                {"InRepairMode", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const bool active = repairCursorUp() && gh && gh->isVendorWindowOpen();
            lua_pushboolean(L, active ? 1 : 0);
            return 1;
        }},
                // IsInventoryItemLocked(slot) - that slot's item is on the
                // cursor, so the paperdoll greys it while it is in the air.
                // Answering no meant a picked-up item stayed drawn in the slot
                // it had already left.
                //
                // It does show, despite SetDesaturated being a no-op here:
                // SetItemButtonDesaturated reads the return value as
                // "shaderSupported" and greys with SetVertexColor(0.5) when it
                // is falsy, which is the branch a no-op takes.
                {"IsInventoryItemLocked",   [](lua_State* L) -> int {
            const int slot = static_cast<int>(luaL_optnumber(L, 1, 0));
            lua_pushboolean(L, slot > 0 && cursorEquipSlot() == slot);
            return 1;
        }},
                // GetInventoryItemBroken(unit, slot) - worn out, so the
                // paperdoll draws that slot's icon red. The durability is
                // already tracked and already answered by
                // GetInventoryItemDurability; only this was left saying no.
                {"GetInventoryItemBroken",  [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int slotId = static_cast<int>(luaL_optnumber(L, 2, 0));
            if (!gh || slotId < 1 || slotId > 19) { lua_pushboolean(L, 0); return 1; }
            const auto& sl = gh->getInventory().getEquipSlot(
                static_cast<game::EquipSlot>(slotId - 1));
            lua_pushboolean(L, !sl.empty() && sl.item.maxDurability > 0 &&
                               sl.item.curDurability == 0);
            return 1;
        }},
                // CursorCanGoInSlot(slot) - whether what the cursor is holding
                // could be worn in that paperdoll slot, which is what makes the
                // slot light up while an item is being dragged. Answering no
                // for everything meant nothing ever lit up.
                //
                // Which slot of a pair, this does not decide: both rings and
                // both trinkets light up, as they do in WoW. That is a
                // different question from the one InventoryScreen answers when
                // it picks a slot to equip into, so this does not go looking
                // for that logic.
                {"CursorCanGoInSlot",       [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int slot = static_cast<int>(luaL_optnumber(L, 1, 0));
            const uint32_t held = cursorItemId();
            if (!gh || slot < 1 || held == 0) { lua_pushboolean(L, 0); return 1; }
            const auto* info = gh->getItemInfo(held);
            if (!info || !info->valid) { lua_pushboolean(L, 0); return 1; }
            // Paperdoll slot ids are the EquipSlot enum plus one.
            using namespace game::InvType;
            bool fits = false;
            switch (info->inventoryType) {
                case HEAD:       fits = (slot == 1);  break;
                case NECK:       fits = (slot == 2);  break;
                case SHOULDERS:  fits = (slot == 3);  break;
                case SHIRT:      fits = (slot == 4);  break;
                case CHEST: case ROBE: fits = (slot == 5); break;
                case WAIST:      fits = (slot == 6);  break;
                case LEGS:       fits = (slot == 7);  break;
                case FEET:       fits = (slot == 8);  break;
                case WRISTS:     fits = (slot == 9);  break;
                case HANDS:      fits = (slot == 10); break;
                case FINGER:     fits = (slot == 11 || slot == 12); break;
                case TRINKET:    fits = (slot == 13 || slot == 14); break;
                case BACK:       fits = (slot == 15); break;
                // A one-hander goes in either hand; a main-hand-only weapon and
                // a two-hander only in the first.
                case ONE_HAND:   fits = (slot == 16 || slot == 17); break;
                case MAIN_HAND: case TWO_HAND: fits = (slot == 16); break;
                case SHIELD: case OFF_HAND: case HOLDABLE: fits = (slot == 17); break;
                case RANGED_BOW: case RANGED_GUN: case THROWN: fits = (slot == 18); break;
                case TABARD:     fits = (slot == 19); break;
                default:         fits = false; break;
            }
            lua_pushboolean(L, fits);
            return 1;
        }},
                // Zero rather than false: paperdollframe.lua asks
                // `IsTitleKnown(i) ~= 0`, and `false ~= 0` compares two
                // different types and is therefore true - so every title would
                // have counted as known. This matters live, not just in
                // theory: GetNumTitles answers 192, so the 1..GetNumTitles
                // loop does run and reads this on every title.
                // IsTitleKnown(bit) - the client tracks these already, as a
                // set of bits off the player's known-titles mask. Answering a
                // constant zero meant the paperdoll's title dropdown listed
                // nothing however many the character had earned.
                //
                // A number rather than a boolean, because paperdollframe.lua
                // asks `IsTitleKnown(i) ~= 0` and false compares unequal to
                // zero - which is the trap that made this a zero rather than a
                // false in the first place.
                // The pet tab of the character sheet, which is handed over -
                // PetPaperDollFrame_SetStats and PetExpBar_Update run whenever
                // it is shown, so with a pet out these raised on opening it.
                //
                // The two modifiers are multipliers, so one is the neutral
                // answer; zero would have read as a pet with no health at all.
                {"GetUnitHealthModifier",   [](lua_State* L) -> int {
            lua_pushnumber(L, 1.0); return 1; }},
                {"GetUnitPowerModifier",    [](lua_State* L) -> int {
            lua_pushnumber(L, 1.0); return 1; }},
                {"GetPetExperience", [](lua_State* L) -> int {
            // currXP, nextXP - off the pet unit's own fields, not the player's.
            auto* gh = getGameHandler(L);
            lua_pushnumber(L, gh ? gh->getPetExperience() : 0);
            lua_pushnumber(L, gh ? gh->getPetNextLevelExp() : 0);
            return 2;
        }},
                // GetCompanionCooldown(mode, index) → start, duration, enabled.
                //
                // A companion's cooldown is its summon spell's, the same
                // relationship GetTradeSkillCooldown reads for a recipe. The
                // comment that used to sit here said this client does not
                // enumerate companions, which stopped being true when
                // rebuildCompanions was written - GetCompanionInfo has listed
                // them, and CallCompanion below has summoned them, ever since.
                //
                // (start, duration), not (now, remaining): the cooldown frame
                // draws a sweep of `duration` beginning at `start`, so the
                // start is wound back by however much has already run.
                {"GetCompanionCooldown", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* kind = luaL_optstring(L, 1, "");
            const int index = static_cast<int>(luaL_optnumber(L, 2, 0));
            if (!gh || index < 1) {
                lua_pushnumber(L, 0); lua_pushnumber(L, 0); lua_pushnumber(L, 1);
                return 3;
            }
            const auto& list = gh->getCompanions(std::string(kind) == "MOUNT");
            float left = 0.0f, total = 0.0f;
            if (index <= static_cast<int>(list.size())) {
                const uint32_t spellId = list[static_cast<size_t>(index) - 1].spellId;
                left = gh->getSpellCooldown(spellId);
                total = gh->getSpellCooldownTotal(spellId);
                if (total < left) total = left;
            }
            if (left <= 0.01f) {
                lua_pushnumber(L, 0); lua_pushnumber(L, 0); lua_pushnumber(L, 1);
                return 3;
            }
            lua_pushnumber(L, luaGetTimeNow() - (total - left));
            lua_pushnumber(L, total);
            lua_pushnumber(L, 1);
            return 3;
        }},
                // Summoning a mount or a critter is casting its spell - there
                // is no separate companion message on the wire in 3.3.5.
                {"CallCompanion", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* kind = luaL_optstring(L, 1, "");
            const int index = static_cast<int>(luaL_optnumber(L, 2, 0));
            if (!gh || index < 1) return 0;
            const auto& list = gh->getCompanions(std::string(kind) == "MOUNT");
            if (index <= static_cast<int>(list.size())) {
                gh->castSpell(list[static_cast<size_t>(index) - 1].spellId, 0);
            }
            return 0;
        }},
                // Putting one away is cancelling its aura, which is the same
                // thing right-clicking the buff does.
                {"DismissCompanion", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* kind = luaL_optstring(L, 1, "");
            if (!gh) return 0;
            const auto& list = gh->getCompanions(std::string(kind) == "MOUNT");
            for (const auto& c : list) {
                for (const auto& a : gh->getPlayerAuras()) {
                    if (a.spellId != c.spellId) continue;
                    gh->cancelAura(c.spellId);
                    return 0;
                }
            }
            return 0;
        }},
                {"IsTitleKnown",            [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int bit = static_cast<int>(luaL_optnumber(L, 1, -1));
            const bool known = gh && bit >= 0 &&
                               gh->getKnownTitleBits().count(static_cast<uint32_t>(bit)) > 0;
            lua_pushnumber(L, known ? 1 : 0);
            return 1; }},
                {"GetCombatRatingBonus",    lua_GetCombatRatingBonus},
                {"GetCritChanceFromAgility", lua_GetCritChanceFromAgility},
                {"GetSpellCritChanceFromIntellect", lua_GetSpellCritChanceFromIntellect},
                // Three values - main hand, off hand, ranged - because the
                // character sheet reads the second and concatenates it. One
                // value left it nil, and the line that prints "expertise /
                // off-hand expertise" raised rather than printing.
                // GetExpertise() → expertise points, main hand and off hand.
                //
                // Two values, not three: the character sheet destructures
                // `local expertise, offhandExpertise = GetExpertise()` and
                // prints them either side of a slash when an off hand is being
                // swung. The third was never read, and both of the first two
                // were zero for everyone - the server sends them, in points,
                // and nothing here was reading the fields.
                {"GetExpertise", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushnumber(L, gh ? gh->getExpertise() : 0);
            lua_pushnumber(L, gh ? gh->getOffhandExpertise() : 0);
            return 2;
        }},
                // Two, for the same reason and under the same guard, four
                // lines below the call above: the character sheet formats the
                // off-hand percentage whenever an off-hand is equipped, and
                // format("%.2f", nil) raises. Fixing GetExpertise alone left
                // every dual-wielder's stat panel broken on the next line.
                // A point of expertise is a quarter of a percent off the
                // chance to be dodged or parried, which is the arithmetic the
                // sheet's tooltip line does not do for itself - it formats
                // whatever it is given with two decimal places and a % sign.
                {"GetExpertisePercent", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushnumber(L, (gh ? gh->getExpertise() : 0) * 0.25);
            lua_pushnumber(L, (gh ? gh->getOffhandExpertise() : 0) * 0.25);
            return 2;
        }},
                {"GetArmorPenetration",     lua_GetArmorPenetration},
                // Spell penetration and shield block value are summed from equipped-item
                // stat mods (ITEM_MOD_SPELL_PENETRATION, the shield's block value), not
                // any player field the server sends - the inventory does not yet total
                // item stat mods, so these stay a genuine zero rather than a stale stub.
                {"GetSpellPenetration",     lua_ZeroPercent},
                {"GetShieldBlock",          lua_ZeroPercent},
                {"GetUnitHealthRegenRateFromSpirit", lua_GetUnitHealthRegenRateFromSpirit},
                {"GetUnitManaRegenRateFromSpirit",   lua_GetUnitManaRegenRateFromSpirit},
                {"GetWatchedFactionInfo",   lua_GetWatchedFactionInfo},
                {"GetNumBagSlots",          lua_GetNumBagSlots},
                {"GetMirrorTimerProgress",  lua_GetMirrorTimerProgress},
                {"GetRestState",            lua_GetRestState},
                {"HasFocus", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushboolean(L, gh && gh->hasFocus() ? 1 : 0);
            return 1;
        }},
    };
    for (const auto& [name, func] : api) {
        lua_pushcfunction(L, func);
        lua_setglobal(L, name);
    }
}

} // namespace wowee::addons
