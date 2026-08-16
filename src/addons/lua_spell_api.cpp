// lua_spell_api.cpp - Spell info, casting, auras, and targeting Lua API bindings.
// Extracted from lua_engine.cpp as part of §5.1 (Tame LuaEngine).
#include "game/shapeshift_forms.hpp"
#include "addons/lua_api_helpers.hpp"
#include "game/item_text.hpp"
#include "addons/lua_engine.hpp"

namespace wowee::addons {

namespace {

/// The highest-rank known spell with this name, or 0 when the player knows
/// none by that name.
///
/// This is what `/cast Frostbolt` means: the name alone picks the best rank the
/// player has. Two bindings resolved it with their own copy of the walk, and
/// picking the wrong one is silent - the spell goes off, it is simply the
/// rank-one version.
///
/// Rank arrives from the DBC as text, "Rank 4", so the number is parsed out of
/// it. A spell with no rank string, which is most of them, sorts as rank zero:
/// right, because it is the only one of its name.
uint32_t highestKnownRankByName(game::GameHandler* gh, const std::string& name) {
    std::string wanted(name);
    toLowerInPlace(wanted);

    uint32_t bestId = 0;
    int bestRank = -1;
    for (uint32_t sid : gh->getKnownSpells()) {
        std::string known = gh->getSpellName(sid);
        toLowerInPlace(known);
        if (known != wanted) continue;

        int rank = 0;
        const std::string& rankText = gh->getSpellRank(sid);
        if (!rankText.empty()) {
            std::string lowered = rankText;
            toLowerInPlace(lowered);
            if (lowered.rfind("rank ", 0) == 0) {
                try { rank = std::stoi(lowered.substr(5)); } catch (...) {}
            }
        }
        if (rank > bestRank) { bestRank = rank; bestId = sid; }
    }
    return bestId;
}

}  // namespace


// ---- Finishing a spell that is waiting for a target ----
//
// A spell cast with no target leaves the cursor holding it until something is
// clicked; these are what the unit and party frames call when that click lands
// on them. This client resolves a target at cast time instead - there is no
// pending spell on the cursor to complete - so each answers false: nothing was
// waiting, so nothing was consumed, and the frame falls through to its ordinary
// click handling rather than swallowing it.
static int lua_SpellTargetUnit(lua_State* L) { lua_pushboolean(L, 0); return 1; }
static int lua_SpellTargetItem(lua_State* L) { lua_pushboolean(L, 0); return 1; }

// Dropping what the cursor holds onto a unit, which opens a trade. The cursor
// carries no item here, so there is nothing to drop.
static int lua_DropItemOnUnit(lua_State* L) { lua_pushboolean(L, 0); return 1; }

// The totem bar's per-slot spell choice, which needs the multi-cast action bar
// this client does not model.
static int lua_SetMultiCastSpell(lua_State* L) { (void)L; return 0; }


// --- Totems ---
//
// Four slots, each holding a spell and how long it has left. The interface
// wants the moment one was placed rather than its age, on the same clock
// GetTime() reports - a bar that is drawn from start and duration cannot use a
// figure measured from a different zero.

// GetTotemInfo(slot) → haveTotem, name, startTime, duration, icon
static int lua_GetTotemInfo(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int slot = static_cast<int>(luaL_optnumber(L, 1, 0));
    if (!gh || slot < 1 || slot > game::GameHandler::NUM_TOTEM_SLOTS) {
        lua_pushboolean(L, 0);
        lua_pushstring(L, "");
        lua_pushnumber(L, 0);
        lua_pushnumber(L, 0);
        lua_pushnil(L);
        return 5;
    }
    const auto& totem = gh->getTotemSlot(slot - 1);
    const bool active = totem.active();
    const double durationSec = totem.durationMs / 1000.0;
    const double remainingSec = totem.remainingMs() / 1000.0;

    lua_pushboolean(L, active ? 1 : 0);
    lua_pushstring(L, active ? gh->getSpellName(totem.spellId).c_str() : "");
    // Placed at however long ago it has already run for.
    lua_pushnumber(L, active ? luaGetTimeNow() - (durationSec - remainingSec) : 0.0);
    lua_pushnumber(L, active ? durationSec : 0.0);
    if (active) {
        const std::string icon = gh->getSpellIconPath(totem.spellId);
        if (!icon.empty()) lua_pushstring(L, icon.c_str()); else lua_pushnil(L);
    } else {
        lua_pushnil(L);
    }
    return 5;
}

// GetTotemTimeLeft(slot) → seconds
static int lua_GetTotemTimeLeft(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int slot = static_cast<int>(luaL_optnumber(L, 1, 0));
    if (!gh || slot < 1 || slot > game::GameHandler::NUM_TOTEM_SLOTS) {
        lua_pushnumber(L, 0);
        return 1;
    }
    lua_pushnumber(L, gh->getTotemSlot(slot - 1).remainingMs() / 1000.0);
    return 1;
}

static int lua_IsSpellInRange(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { return luaReturnNil(L); }
    const char* spellNameOrId = luaL_checkstring(L, 1);
    const char* uid = luaL_optstring(L, 2, "target");

    // Resolve spell ID
    uint32_t spellId = 0;
    if (spellNameOrId[0] >= '0' && spellNameOrId[0] <= '9') {
        spellId = static_cast<uint32_t>(strtoul(spellNameOrId, nullptr, 10));
    } else {
        // The rank that would actually be cast, not the first one found. A
        // range check exists to say whether pressing the button will work, so
        // it has to be asked of the same spell /cast would pick; where ranks
        // differ in range, answering about rank one greys out a button that
        // would have reached.
        spellId = highestKnownRankByName(gh, spellNameOrId);
    }
    if (spellId == 0) { return luaReturnNil(L); }

    // Get spell max range from DBC
    auto data = gh->getSpellData(spellId);
    if (data.maxRange <= 0.0f) { return luaReturnNil(L); }

    // Resolve target position
    std::string uidStr(uid);
    toLowerInPlace(uidStr);
    uint64_t guid = resolveUnitGuid(gh, uidStr);
    if (guid == 0) { return luaReturnNil(L); }
    auto targetEnt = gh->getEntityManager().getEntity(guid);
    auto playerEnt = gh->getEntityManager().getEntity(gh->getPlayerGuid());
    if (!targetEnt || !playerEnt) { return luaReturnNil(L); }

    float dx = playerEnt->getX() - targetEnt->getX();
    float dy = playerEnt->getY() - targetEnt->getY();
    float dz = playerEnt->getZ() - targetEnt->getZ();
    float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
    lua_pushnumber(L, dist <= data.maxRange ? 1 : 0);
    return 1;
}

// UnitIsVisible(unit) → boolean (entity exists in the client's entity manager)

/// Whether the player's class can remove a debuff of this dispel type.
///
/// The "RAID" filter FrameXML passes on party frames means "only what I can do
/// something about", and it is what showDispelDebuffs turns on. Without it the
/// filter string was accepted and ignored, so the frames showed every debuff
/// however little the healer could do - and answering the CVar without this
/// would have claimed a filter that does not filter.
///
/// Dispel types are Spell.dbc's: 1 Magic, 2 Curse, 3 Disease, 4 Poison. The
/// class table is 3.3.5's, and it is about what the class can remove from a
/// *friendly* target rather than what it can purge from an enemy.
static bool classCanDispel(uint8_t classId, uint8_t dispelType) {
    if (dispelType == 0) return false;
    constexpr uint8_t kMagic = 1, kCurse = 2, kDisease = 3, kPoison = 4;
    switch (classId) {
        case 2:  // Paladin - Cleanse
            return dispelType == kMagic || dispelType == kPoison || dispelType == kDisease;
        case 5:  // Priest - Dispel Magic, Abolish Disease
            return dispelType == kMagic || dispelType == kDisease;
        case 7:  // Shaman - Cure Toxins, Cleanse Spirit at 51
            return dispelType == kCurse || dispelType == kPoison || dispelType == kDisease;
        case 8:  // Mage - Remove Curse
            return dispelType == kCurse;
        case 11: // Druid - Remove Curse, Abolish Poison
            return dispelType == kCurse || dispelType == kPoison;
        case 6:  // Death Knight - the pet's Leech, and unholy's disease removal
            return dispelType == kDisease;
        case 9:  // Warlock - the Felhunter's Devour Magic
            return dispelType == kMagic;
        default: // Warrior, Hunter, Rogue: nothing
            return false;
    }
}

static int lua_UnitAura(lua_State* L, bool wantBuff) {
    auto* gh = getGameHandler(L);
    if (!gh) { return luaReturnNil(L); }
    const char* uid = luaL_optstring(L, 1, "player");
    int index = static_cast<int>(luaL_optnumber(L, 2, 1));
    if (index < 1) { return luaReturnNil(L); }

    std::string uidStr(uid);
    toLowerInPlace(uidStr);

    const std::vector<game::AuraSlot>* auras = nullptr;
    if (uidStr == "player")      auras = &gh->getPlayerAuras();
    else if (uidStr == "target") auras = &gh->getTargetAuras();
    else {
        // Try party/raid/focus via GUID lookup in unitAurasCache
        uint64_t guid = resolveUnitGuid(gh, uidStr);
        if (guid != 0) auras = gh->getUnitAuras(guid);
    }
    if (!auras) { return luaReturnNil(L); }

    // "RAID" on a debuff means only what this character can remove. The index
    // counts into the filtered list, so the test has to come before the count
    // - filtering after it would answer the wrong aura for every index past
    // the first one skipped.
    const char* filterArg = luaL_optstring(L, 3, "");
    bool dispellableOnly = false;
    if (!wantBuff && filterArg) {
        std::string f(filterArg);
        for (char& c : f) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
        dispellableOnly = f.find("RAID") != std::string::npos;
    }
    const uint8_t playerClass = gh->getPlayerClass();

    // Filter to buffs or debuffs and find the Nth one
    int found = 0;
    for (const auto& aura : *auras) {
        if (aura.isEmpty() || aura.spellId == 0) continue;
        bool isDebuff = (aura.flags & 0x80) != 0;
        if (wantBuff ? isDebuff : !isDebuff) continue;
        if (dispellableOnly &&
            !classCanDispel(playerClass, gh->getSpellDispelType(aura.spellId))) {
            continue;
        }
        found++;
        if (found == index) {
            // Return: name, rank, icon, count, debuffType, duration, expirationTime, ...spellId
            std::string name = gh->getSpellName(aura.spellId);
            lua_pushstring(L, name.empty() ? "Unknown" : name.c_str()); // name
            lua_pushstring(L, "");           // rank
            std::string iconPath = gh->getSpellIconPath(aura.spellId);
            if (!iconPath.empty()) lua_pushstring(L, iconPath.c_str());
            else lua_pushnil(L);             // icon texture path
            lua_pushnumber(L, aura.charges); // count
            // debuffType: resolve from Spell.dbc dispel type
            {
                uint8_t dt = gh->getSpellDispelType(aura.spellId);
                switch (dt) {
                    case 1:  lua_pushstring(L, "Magic");  break;
                    case 2:  lua_pushstring(L, "Curse");  break;
                    case 3:  lua_pushstring(L, "Disease"); break;
                    case 4:  lua_pushstring(L, "Poison"); break;
                    default: lua_pushnil(L);              break;
                }
            }
            lua_pushnumber(L, aura.maxDurationMs > 0 ? aura.maxDurationMs / 1000.0 : 0); // duration
            // expirationTime: GetTime() + remaining seconds (so addons can compute countdown)
            if (aura.durationMs > 0) {
                uint64_t auraNowMs = static_cast<uint64_t>(
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now().time_since_epoch()).count());
                int32_t remMs = aura.getRemainingMs(auraNowMs);
                lua_pushnumber(L, luaGetTimeNow() + remMs / 1000.0);
            } else {
                lua_pushnumber(L, 0);  // permanent aura
            }
            // caster: return unit ID string if caster is known
            if (aura.casterGuid != 0) {
                if (aura.casterGuid == gh->getPlayerGuid())
                    lua_pushstring(L, "player");
                else if (aura.casterGuid == gh->getTargetGuid())
                    lua_pushstring(L, "target");
                else if (aura.casterGuid == gh->getFocusGuid())
                    lua_pushstring(L, "focus");
                else if (aura.casterGuid == gh->getPetGuid())
                    lua_pushstring(L, "pet");
                else {
                    char cBuf[32];
                    snprintf(cBuf, sizeof(cBuf), "0x%016llX", (unsigned long long)aura.casterGuid);
                    lua_pushstring(L, cBuf);
                }
            } else {
                lua_pushnil(L);
            }
            lua_pushboolean(L, 0);           // isStealable
            lua_pushboolean(L, 0);           // shouldConsolidate
            lua_pushnumber(L, aura.spellId); // spellId
            return 11;
        }
    }
    lua_pushnil(L);
    return 1;
}

static int lua_UnitBuff(lua_State* L) { return lua_UnitAura(L, true); }
static int lua_UnitDebuff(lua_State* L) { return lua_UnitAura(L, false); }

// UnitAura(unit, index, filter) - generic aura query with filter string
// filter: "HELPFUL" = buffs, "HARMFUL" = debuffs, "PLAYER" = cast by player,
//         "HELPFUL|PLAYER" = buffs cast by player, etc.
static int lua_UnitAuraGeneric(lua_State* L) {
    const char* filter = luaL_optstring(L, 3, "HELPFUL");
    std::string f(filter ? filter : "HELPFUL");
    for (char& c : f) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    bool wantBuff = (f.find("HARMFUL") == std::string::npos);
    return lua_UnitAura(L, wantBuff);
}

// ---------- UnitCastingInfo / UnitChannelInfo ----------
// Internal helper: pushes cast/channel info for a unit.
// Returns number of Lua return values (0 if not casting/channeling the requested type).
static int lua_UnitCastInfo(lua_State* L, bool wantChannel) {
    auto* gh = getGameHandler(L);
    if (!gh) { return luaReturnNil(L); }

    const char* uid = luaL_optstring(L, 1, "player");
    std::string uidStr(uid ? uid : "player");

    // Use shared GetTime() epoch for consistent timestamps
    double nowSec = luaGetTimeNow();

    // Resolve cast state for the unit
    bool isCasting = false;
    bool isChannel = false;
    uint32_t spellId = 0;
    float timeTotal = 0.0f;
    float timeRemaining = 0.0f;
    bool interruptible = true;

    if (uidStr == "player") {
        isCasting = gh->isCasting();
        isChannel = gh->isChanneling();
        spellId = gh->getCurrentCastSpellId();
        timeTotal = gh->getCastTimeTotal();
        timeRemaining = gh->getCastTimeRemaining();
        // Player interruptibility: always true for own casts (server controls actual interrupt)
        interruptible = true;
    } else {
        uint64_t guid = resolveUnitGuid(gh, uidStr);
        if (guid == 0) { return luaReturnNil(L); }
        const auto* state = gh->getUnitCastState(guid);
        if (!state) { return luaReturnNil(L); }
        isCasting = state->casting;
        isChannel = state->isChannel;
        spellId = state->spellId;
        timeTotal = state->timeTotal;
        timeRemaining = state->timeRemaining;
        interruptible = state->interruptible;
    }

    if (!isCasting) { return luaReturnNil(L); }

    // UnitCastingInfo: only returns for non-channel casts
    // UnitChannelInfo: only returns for channels
    if (wantChannel != isChannel) { return luaReturnNil(L); }

    // Spell name + icon
    const std::string& name = gh->getSpellName(spellId);
    std::string iconPath = gh->getSpellIconPath(spellId);

    // Time values in milliseconds (WoW API convention)
    double startTimeMs = (nowSec - (timeTotal - timeRemaining)) * 1000.0;
    double endTimeMs   = (nowSec + timeRemaining) * 1000.0;

    // The real signatures, which are one wider than what this used to send:
    //
    //   UnitCastingInfo → name, nameSubtext, text, texture, startTime,
    //                     endTime, isTradeSkill, castID, notInterruptible
    //   UnitChannelInfo → the same without castID
    //
    // nameSubtext is the rank, and leaving it out shifted every value after it
    // by one. castingbarframe.lua destructures all nine by name, so it read the
    // icon path as the bar's text, a timestamp as the texture, and a boolean as
    // endTime - then computed (endTime - startTime) and raised, which is why no
    // cast bar was ever drawn rather than a wrong one.
    lua_pushstring(L, name.empty() ? "Unknown" : name.c_str()); // name
    const std::string& rank = gh->getSpellRank(spellId);
    lua_pushstring(L, rank.c_str());                             // nameSubtext
    lua_pushstring(L, "");                                       // text
    if (!iconPath.empty()) lua_pushstring(L, iconPath.c_str());
    else lua_pushstring(L, "Interface\\Icons\\INV_Misc_QuestionMark");  // texture
    lua_pushnumber(L, startTimeMs);                              // startTime (ms)
    lua_pushnumber(L, endTimeMs);                                // endTime (ms)
    lua_pushboolean(L, gh->isProfessionSpell(spellId) ? 1 : 0); // isTradeSkill
    if (!wantChannel) {
        lua_pushnumber(L, spellId);                              // castID (UnitCastingInfo only)
    }
    lua_pushboolean(L, interruptible ? 0 : 1);                  // notInterruptible
    return wantChannel ? 8 : 9;
}

static int lua_UnitCastingInfo(lua_State* L) { return lua_UnitCastInfo(L, false); }
static int lua_UnitChannelInfo(lua_State* L) { return lua_UnitCastInfo(L, true); }

static int lua_CastSpellByName(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) return 0;
    const char* name = luaL_checkstring(L, 1);
    if (!name || !*name) return 0;

    const uint32_t bestId = highestKnownRankByName(gh, name);
    if (bestId != 0) {
        // The same second argument, and the same branch of SECURE_ACTIONS.spell
        // reaches here: a spell named rather than numbered is cast by name, and
        // the unit clicked still says who it lands on.
        uint64_t target = 0;
        if (const char* unit = luaL_optstring(L, 2, nullptr)) {
            std::string u(unit);
            toLowerInPlace(u);
            target = resolveUnitGuid(gh, u);
        }
        if (target == 0 && gh->hasTarget()) target = gh->getTargetGuid();
        gh->castSpell(bestId, target);
    }
    return 0;
}

static int lua_IsSpellKnown(lua_State* L) {
    auto* gh = getGameHandler(L);
    uint32_t spellId = static_cast<uint32_t>(luaL_checknumber(L, 1));
    lua_pushboolean(L, gh && gh->getKnownSpells().count(spellId));
    return 1;
}

// --- Spell Book Tab API ---

// GetNumSpellTabs() → count
static int lua_GetNumSpellTabs(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { return luaReturnZero(L); }
    lua_pushnumber(L, gh->getSpellBookTabs().size());
    return 1;
}

// GetSpellTabInfo(tabIndex) → name, texture, offset, numSpells,
//                             highestRankOffset, highestRankNumSpells
// tabIndex is 1-based; offset is 1-based global spell book slot
//
// Six values, and the last two are not optional. SpellBook_GetTabInfo unpacks
// all six and then, when ShowAllSpellRanks is off - which is the default -
// throws away offset and numSpells and uses the highest-rank pair instead. So
// a four-value answer leaves both nil on the ordinary path and the spellbook
// raises on the arithmetic, showing nothing at all.
//
// The comment here used to say four. Cutting the returns to match it would
// have emptied the spellbook, which is exactly what a stubbed four-value
// version did when it was tried.
static int lua_GetSpellTabInfo(lua_State* L) {
    auto* gh = getGameHandler(L);
    int tabIdx = static_cast<int>(luaL_checknumber(L, 1));
    // An empty tab rather than one nil, because the caller does arithmetic on
    // what comes back without checking it.
    //
    // SpellBookFrame's own OnLoad ends with SpellBookSkillLineTab_OnClick(nil,
    // 1), and three frames down that divides numSpells by the page size. In
    // WoW tab one is General and always exists, so FrameXML never guards it -
    // here it does not exist until the spell list has arrived, and a single
    // nil made that a raise inside OnLoad, which loses the whole file.
    // SpellBookFrame.xml was failing to load outright.
    //
    // Six zeroes give a book with no pages, which is what a player with no
    // spells yet has, and the frame loads and fills in when they arrive.
    auto emptyTab = [](lua_State* s) {
        lua_pushstring(s, "");   // name
        lua_pushstring(s, "");   // texture
        lua_pushnumber(s, 0);    // offset
        lua_pushnumber(s, 0);    // numSpells
        lua_pushnumber(s, 0);    // highestRankOffset
        lua_pushnumber(s, 0);    // highestRankNumSpells
        return 6;
    };
    if (!gh || tabIdx < 1) {
        return emptyTab(L);
    }
    const auto& tabs = gh->getSpellBookTabs();
    if (tabIdx > static_cast<int>(tabs.size())) {
        return emptyTab(L);
    }
    // Compute offset: sum of spells in all preceding tabs (1-based)
    int offset = 0;
    for (int i = 0; i < tabIdx - 1; ++i)
        offset += static_cast<int>(tabs[i].spellIds.size());
    const auto& tab = tabs[tabIdx - 1];
    lua_pushstring(L, tab.name.c_str());           // name
    lua_pushstring(L, tab.texture.c_str());        // texture
    lua_pushnumber(L, offset);                     // offset (0-based for WoW compat)
    lua_pushnumber(L, tab.spellIds.size());        // numSpells
    // The highest-rank pair, which is what FrameXML actually reads: with
    // ShowAllSpellRanks off - the default - SpellBook_GetTabInfo throws away
    // the first offset and count and keeps these. Returning four values left
    // numSpells nil and the page count divided by nothing. This client does
    // not track ranks separately, so the whole tab is the highest rank.
    lua_pushnumber(L, offset);                     // highestRankOffset
    lua_pushnumber(L, tab.spellIds.size());        // highestRankNumSpells
    return 6;
}

// GetSpellBookItemInfo(slot, bookType) → "SPELL", spellId
// slot is 1-based global spell book index
static int lua_GetSpellBookItemInfo(lua_State* L) {
    auto* gh = getGameHandler(L);
    int slot = static_cast<int>(luaL_checknumber(L, 1));
    if (!gh || slot < 1) {
        lua_pushstring(L, "SPELL");
        lua_pushnumber(L, 0);
        return 2;
    }
    const auto& tabs = gh->getSpellBookTabs();
    int idx = slot; // 1-based
    for (const auto& tab : tabs) {
        if (idx <= static_cast<int>(tab.spellIds.size())) {
            lua_pushstring(L, "SPELL");
            lua_pushnumber(L, tab.spellIds[idx - 1]);
            return 2;
        }
        idx -= static_cast<int>(tab.spellIds.size());
    }
    lua_pushstring(L, "SPELL");
    lua_pushnumber(L, 0);
    return 2;
}

// GetSpellBookItemName(slot, bookType) → name, subName
static int lua_GetSpellBookItemName(lua_State* L) {
    auto* gh = getGameHandler(L);
    int slot = static_cast<int>(luaL_checknumber(L, 1));
    if (!gh || slot < 1) { return luaReturnNil(L); }
    const auto& tabs = gh->getSpellBookTabs();
    int idx = slot;
    for (const auto& tab : tabs) {
        if (idx <= static_cast<int>(tab.spellIds.size())) {
            uint32_t spellId = tab.spellIds[idx - 1];
            const std::string& name = gh->getSpellName(spellId);
            lua_pushstring(L, name.empty() ? "Unknown" : name.c_str());
            lua_pushstring(L, ""); // subName/rank
            return 2;
        }
        idx -= static_cast<int>(tab.spellIds.size());
    }
    lua_pushnil(L);
    return 1;
}


// ── The spellbook's older API ──────────────────────────────────────────────
//
// SpellBookFrame in WotLK uses both generations at once: the newer
// GetSpellBookItem* alongside GetSpellName and CastSpell, which take the same
// slot and book. The newer half was implemented and the older was not, so the
// panel could name a spell and not cast it.


/// GetSpellName(slot, bookType) → name, rank.
static int lua_GetSpellName(lua_State* L) {
    auto* gh = getGameHandler(L);
    // Through spellIdForCall, which reads the book as well as the slot. The
    // spellbook passes SpellBookFrame.bookType to every one of these, and
    // resolving a pet slot through the player's tabs names one of the player's
    // own spells - the wrong answer wearing the shape of a right one.
    const uint32_t id = spellIdForCall(L, gh);
    if (id == 0) { return luaReturnNil(L); }
    const std::string& name = gh->getSpellName(id);
    lua_pushstring(L, name.empty() ? "Unknown" : name.c_str());
    lua_pushstring(L, "");
    return 2;
}

/// CastSpell(slot, bookType) - cast what is in that slot at the current
/// target, which is what clicking a spellbook entry does.
static int lua_CastSpell(lua_State* L) {
    auto* gh = getGameHandler(L);
    // Same resolver, and here it decides what is cast: clicking a pet spell
    // resolved through the player's tabs cast whichever of the player's spells
    // sat at that index.
    const uint32_t id = spellIdForCall(L, gh);
    if (id != 0) gh->castSpell(id, gh->getTargetGuid());
    return 0;
}

/// IsPassiveSpell(slot, bookType) → whether the spell is passive, which is how
/// the book decides not to draw a cast button for it. Spell attributes are not
/// read from the DBC yet, so this answers no - which draws a button for a
/// passive rather than hiding a real one, the less wrong way round.
/// IsPassiveSpell(id or index, bookType) → whether the spell is never cast.
///
/// It answered no for everything, so the spell book drew a passive with the
/// same cast border as an ability and let it be dragged onto the action bar,
/// where it would sit doing nothing. Spell.dbc's base attribute word carries
/// it - bit 6 - beside the Ex word the client already read for
/// interruptibility.
static int lua_IsPassiveSpell(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { lua_pushboolean(L, 0); return 1; }
    // Both forms: IsPassiveSpell(spellId) and IsPassiveSpell(slot, bookType),
    // which is the one the spellbook uses - and reading a slot as an id
    // answered for whichever spell happens to carry the number 1, 2 or 3.
    // The result decides whether a spell is drawn as castable at all.
    const uint32_t id = spellIdForCall(L, gh);
    lua_pushboolean(L, gh->isSpellPassive(id) ? 1 : 0);
    return 1;
}

/// IsSelectedSpell(slot, bookType) → whether the book is highlighting it.
/// Nothing selects a spell here; the highlight follows the cursor.
static int lua_IsSelectedSpell(lua_State* L) {
    lua_pushboolean(L, 0);
    return 1;
}

/// GetSpellAutocast(slot, bookType) → autocastable, currently autocasting.
/// Pet spells only, and pet action bars are not modelled: false says the
/// little rotating border is not drawn, which is right for every spell a
/// player has.
static int lua_GetSpellAutocast(lua_State* L) {
    lua_pushboolean(L, 0);
    lua_pushboolean(L, 0);
    return 2;
}
/// GetKnownSlotFromHighestRankSlot(slot) → the slot actually known. The book
/// shows the highest rank and this maps back; with one entry per spell here
/// the two are the same slot.
static int lua_GetKnownSlotFromHighestRankSlot(lua_State* L) {
    lua_pushnumber(L, luaL_optnumber(L, 1, 0));
    return 1;
}

/// UpdateSpells() - redraw the spell book page.
///
/// The list itself needs no rebuilding, which is why this was a no-op. But in
/// WoW the call is what makes the client announce SPELLS_CHANGED, and that
/// event is the only thing that redraws a spell button: SpellButton_OnEvent
/// answers it with SpellButton_UpdateButton, and nothing else calls that.
///
/// Six places in spellbookframe.lua call this, and every one of them is a
/// "now redraw the page" - after a skill-line tab is clicked, after a page
/// turn, after the book type changes. SpellBookFrame_ShowSpells only calls
/// Show() on buttons that are already shown, so no OnShow fires and without
/// the event the page kept whatever it was displaying before. That is the
/// whole of "the spell book does not update when the tab changes": the offset
/// moved and nothing ever re-read it.
static int lua_UpdateSpells(lua_State* L) {
    lua_getfield(L, LUA_REGISTRYINDEX, "wowee_lua_engine");
    auto* engine = static_cast<LuaEngine*>(lua_touserdata(L, -1));
    lua_pop(L, 1);
    // Safe against the obvious loop: SpellBookFrame_OnEvent answers
    // SPELLS_CHANGED with SpellBookFrame_Update() and no argument, and the
    // branch that would call back here only runs when it is passed one.
    if (engine) engine->fireEvent("SPELLS_CHANGED", {});
    return 0;
}


static int lua_GetSpellDescription(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { lua_pushstring(L, ""); return 1; }
    uint32_t spellId = static_cast<uint32_t>(luaL_checknumber(L, 1));
    // The shared formatter, which is what every other description path uses.
    // The local one this replaced knew $s and $o and wrote a literal "X" for
    // $d and the rest, so a duration read as "lasts X sec".
    const std::string desc =
        gh->formatSpellDescription(spellId, gh->getSpellDescription(spellId));
    lua_pushstring(L, desc.c_str());
    return 1;
}


static int lua_GetEnchantInfo(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { return luaReturnNil(L); }
    uint32_t enchantId = static_cast<uint32_t>(luaL_checknumber(L, 1));
    std::string name = gh->getEnchantName(enchantId);
    if (name.empty()) { return luaReturnNil(L); }
    lua_pushstring(L, name.c_str());
    return 1;
}

static int lua_GetSpellCooldown(lua_State* L) {
    auto* gh = getGameHandler(L);
    // Three values on this path too, like every other. The cooldown frame does
    // `start > 0 and duration > 0 and enable > 0`, and `and` short-circuits, so
    // a missing third value is only ever reached when the first two say a
    // cooldown is running - which this path cannot say, because it answers
    // zero. Safe by accident rather than by design, and the next person to give
    // this branch a real start time would have found out the hard way.
    if (!gh) { lua_pushnumber(L, 0); lua_pushnumber(L, 0); lua_pushnumber(L, 1); return 3; }
    // A name, an id, or a book slot with the book beside it - the spellbook
    // asks in the last of those three.
    const uint32_t spellId = spellIdForCall(L, gh);
    float cd = gh->getSpellCooldown(spellId);
    // Also check GCD - if spell has no individual cooldown but GCD is active,
    // return the GCD timing (this is how WoW handles it)
    float gcdRem = gh->getGCDRemaining();
    float gcdTotal = gh->getGCDTotal();

    // WoW returns (start, duration, enabled) where remaining = start + duration - GetTime()
    double nowSec = luaGetTimeNow();

    if (cd > 0.01f) {
        // Spell-specific cooldown (longer than GCD). The pair asked for is
        // (start, duration), not (now, remaining) - the cooldown frame draws a
        // sweep of `duration` beginning at `start`, so saying it began now with
        // a length of whatever is left restarts the swirl at full every time
        // the interface asks. It asks on every ACTIONBAR_UPDATE_COOLDOWN, which
        // fires on every cast, so a five-minute cooldown looked like it kept
        // starting over. Wound back the same way the GCD branch below already
        // does it.
        double total = gh->getSpellCooldownTotal(spellId);
        if (total < cd) total = cd;   // a total never recorded falls back to what is left
        double start = nowSec - (total - cd);
        lua_pushnumber(L, start);
        lua_pushnumber(L, total);
    } else if (gcdRem > 0.01f) {
        // GCD is active - return GCD timing
        double elapsed = gcdTotal - gcdRem;
        double start = nowSec - elapsed;
        lua_pushnumber(L, start);
        lua_pushnumber(L, gcdTotal);
    } else {
        lua_pushnumber(L, 0);       // not on cooldown
        lua_pushnumber(L, 0);
    }
    lua_pushnumber(L, 1);           // enabled
    return 3;
}

static int lua_HasTarget(lua_State* L) {
    auto* gh = getGameHandler(L);
    lua_pushboolean(L, gh && gh->hasTarget());
    return 1;
}

// TargetUnit(unitId) - set current target
static int lua_TargetUnit(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) return 0;
    const char* uid = luaL_checkstring(L, 1);
    std::string uidStr(uid);
    toLowerInPlace(uidStr);
    uint64_t guid = resolveUnitGuid(gh, uidStr);
    if (guid != 0) gh->setTarget(guid);
    return 0;
}

// ClearTarget() - clear current target
static int lua_ClearTarget(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (gh) gh->clearTarget();
    return 0;
}

// FocusUnit(unitId) - set focus target
static int lua_FocusUnit(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) return 0;
    const char* uid = luaL_optstring(L, 1, nullptr);
    if (!uid || !*uid) return 0;
    std::string uidStr(uid);
    toLowerInPlace(uidStr);
    uint64_t guid = resolveUnitGuid(gh, uidStr);
    if (guid != 0) gh->setFocus(guid);
    return 0;
}

// ClearFocus() - clear focus target
static int lua_ClearFocus(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (gh) gh->clearFocus();
    return 0;
}

// AssistUnit(unitId) - target whatever the given unit is targeting
static int lua_AssistUnit(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) return 0;
    const char* uid = luaL_optstring(L, 1, "target");
    std::string uidStr(uid);
    toLowerInPlace(uidStr);
    uint64_t guid = resolveUnitGuid(gh, uidStr);
    if (guid == 0) return 0;
    uint64_t theirTarget = getEntityTargetGuid(gh, guid);
    if (theirTarget != 0) gh->setTarget(theirTarget);
    return 0;
}

// TargetLastTarget() - re-target previous target
static int lua_TargetLastTarget(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (gh) gh->targetLastTarget();
    return 0;
}

// TargetNearestEnemy() - tab-target nearest enemy
// The argument is the direction, and it was being thrown away. Bindings.xml
// binds two keys to each of these - `TargetNearestEnemy()` forwards and
// `TargetNearestEnemy(1)` backwards, with a comment on that line saying so -
// so cycling backwards through targets went forwards instead, and the second
// key of every such pair did what the first did.
static bool wantsReverse(lua_State* L) {
    return lua_toboolean(L, 1) != 0;
}

static int lua_TargetNearestEnemy(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (gh) gh->targetEnemy(wantsReverse(L));
    return 0;
}

// TargetNearestFriend([reverse]) - target nearest friendly unit
static int lua_TargetNearestFriend(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (gh) gh->targetFriend(wantsReverse(L));
    return 0;
}

// GetRaidTargetIndex(unit) → icon index (1-8) or nil
static int lua_GetRaidTargetIndex(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { return luaReturnNil(L); }
    const char* uid = luaL_optstring(L, 1, "target");
    std::string uidStr(uid);
    toLowerInPlace(uidStr);
    uint64_t guid = resolveUnitGuid(gh, uidStr);
    if (guid == 0) { return luaReturnNil(L); }
    uint8_t mark = gh->getEntityRaidMark(guid);
    if (mark == 0xFF) { return luaReturnNil(L); }
    lua_pushnumber(L, mark + 1); // WoW uses 1-indexed (1=Star, 2=Circle, ... 8=Skull)
    return 1;
}

// SetRaidTarget(unit, index) - set raid marker (1-8, or 0 to clear)
static int lua_SetRaidTarget(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) return 0;
    const char* uid = luaL_optstring(L, 1, "target");
    int index = static_cast<int>(luaL_checknumber(L, 2));
    std::string uidStr(uid);
    toLowerInPlace(uidStr);
    uint64_t guid = resolveUnitGuid(gh, uidStr);
    if (guid == 0) return 0;
    if (index >= 1 && index <= 8)
        gh->setRaidMark(guid, static_cast<uint8_t>(index - 1));
    else if (index == 0)
        gh->setRaidMark(guid, 0xFF); // clear
    return 0;
}

// GetSpellPowerCost(spellId) → {{ type=powerType, cost=manaCost, name=powerName }}

static int lua_GetSpellPowerCost(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { lua_newtable(L); return 1; }
    uint32_t spellId = static_cast<uint32_t>(luaL_checknumber(L, 1));
    auto data = gh->getSpellData(spellId);
    lua_newtable(L); // outer table (array of cost entries)
    if (data.manaCost > 0) {
        lua_newtable(L); // cost entry
        lua_pushnumber(L, data.powerType);
        lua_setfield(L, -2, "type");
        lua_pushnumber(L, data.manaCost);
        lua_setfield(L, -2, "cost");
    
        lua_pushstring(L, data.powerType < 7 ? kLuaPowerNames[data.powerType] : "MANA");
        lua_setfield(L, -2, "name");
        lua_rawseti(L, -2, 1); // outer[1] = entry
    }
    return 1;
}

// --- GetSpellInfo / GetSpellTexture ---
// GetSpellInfo(spellIdOrName) -> name, rank, icon, castTime, minRange, maxRange, spellId
static int lua_GetSpellInfo(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { return luaReturnNil(L); }

    uint32_t spellId = 0;
    if (lua_isnumber(L, 1)) {
        spellId = static_cast<uint32_t>(lua_tonumber(L, 1));
    } else if (lua_isstring(L, 1)) {
        const char* name = lua_tostring(L, 1);
        if (!name || !*name) { return luaReturnNil(L); }
        spellId = highestKnownRankByName(gh, name);
    }

    if (spellId == 0) { return luaReturnNil(L); }
    std::string name = gh->getSpellName(spellId);
    if (name.empty()) { return luaReturnNil(L); }

    lua_pushstring(L, name.c_str());                        // 1: name
    const std::string& rank = gh->getSpellRank(spellId);
    lua_pushstring(L, rank.c_str());                        // 2: rank
    std::string iconPath = gh->getSpellIconPath(spellId);
    if (!iconPath.empty()) lua_pushstring(L, iconPath.c_str());
    else lua_pushnil(L);                                     // 3: icon texture path
    // The client's order, which this did not follow.
    //
    //   name, rank, icon, cost, isFunnel, powerType, castTime, minRange, maxRange
    //
    // Seven values were returned and only the first three were in the right
    // places. Everything asking for a cost got a cast time in milliseconds,
    // everything asking for a cast time got the spell id - so a spell read as
    // taking several thousand seconds to cast - and the two ranges were nil.
    // The interface unpacks all nine in one line in multicastactionbarframe,
    // and any addon reading a spell does the same.
    //
    // isFunnel is the one value with nothing behind it here. False is right
    // for every spell but a handful of warlock drains, and it is what the
    // callers branch on rather than display.
    auto spellData = gh->getSpellData(spellId);
    lua_pushnumber(L, spellData.manaCost);                   // 4: cost
    lua_pushboolean(L, 0);                                   // 5: isFunnel
    lua_pushnumber(L, spellData.powerType);                  // 6: powerType
    lua_pushnumber(L, spellData.castTimeMs);                 // 7: castTime (ms)
    lua_pushnumber(L, spellData.minRange);                   // 8: minRange (yards)
    lua_pushnumber(L, spellData.maxRange);                   // 9: maxRange (yards)
    return 9;
}

// GetSpellTexture(spellIdOrName) -> icon texture path string
static int lua_GetSpellTexture(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { return luaReturnNil(L); }

    const uint32_t spellId = spellIdForCall(L, gh);
    if (spellId == 0) { return luaReturnNil(L); }
    std::string iconPath = gh->getSpellIconPath(spellId);
    if (!iconPath.empty()) lua_pushstring(L, iconPath.c_str());
    else lua_pushnil(L);
    return 1;
}

// GetItemInfo(itemId) -> name, link, quality, iLevel, reqLevel, class, subclass, maxStack, equipSlot, texture, vendorPrice

static int lua_GetSpellLink(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { return luaReturnNil(L); }

    const uint32_t spellId = spellIdForCall(L, gh);
    if (spellId == 0) { return luaReturnNil(L); }
    std::string name = gh->getSpellName(spellId);
    if (name.empty()) { return luaReturnNil(L); }
    const std::string link = game::spellChatLink(spellId, name);
    lua_pushstring(L, link.c_str());
    return 1;
}

/// CancelUnitBuff(unit, indexOrName [, filterOrRank]) - both forms.
///
/// The interface uses both and this took only the index, through
/// luaL_checknumber - so the two name forms raised on the spot rather than
/// cancelling anything. The possess bar cancels the aura holding the player in
/// a vehicle by name, and /cancelaura passes what was typed; both were an
/// error message where an aura should have dropped.
///
/// The third argument is a filter for the index form and a rank for the name
/// form, which is how WoW spells it. Only HELPFUL is answerable - a debuff
/// cannot be cancelled - so the filter decides nothing here beyond confirming
/// buffs are what is being counted.
static int lua_CancelUnitBuff(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) return 0;
    std::string uidStr(luaL_optstring(L, 1, "player"));
    toLowerInPlace(uidStr);
    if (uidStr != "player") return 0;   // only one's own auras can be cancelled

    const auto& auras = gh->getPlayerAuras();
    const bool byName = lua_isstring(L, 2) && !lua_isnumber(L, 2);

    if (byName) {
        std::string want(lua_tostring(L, 2));
        toLowerInPlace(want);
        for (const auto& a : auras) {
            if (a.isEmpty() || (a.flags & 0x80) != 0) continue;
            std::string have = gh->getSpellName(a.spellId);
            toLowerInPlace(have);
            if (have == want) { gh->cancelAura(a.spellId); return 0; }
        }
        return 0;
    }

    const int index = static_cast<int>(luaL_optnumber(L, 2, 0));
    if (index < 1) return 0;
    int buffCount = 0;
    for (const auto& a : auras) {
        if (a.isEmpty()) continue;
        if ((a.flags & 0x80) != 0) continue;   // debuffs are not cancellable
        if (++buffCount == index) { gh->cancelAura(a.spellId); return 0; }
    }
    return 0;
}

// CastSpellByID(spellId) - cast spell by numeric ID
static int lua_CastSpellByID(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) return 0;
    uint32_t spellId = static_cast<uint32_t>(luaL_checknumber(L, 1));
    if (spellId == 0) return 0;
    // The second argument names who it is cast on, and this is the click-cast
    // path: SECURE_ACTIONS.spell does CastSpellByID(spellID, unit) with the
    // unit off the button that was clicked. Ignoring it cast on whatever was
    // targeted instead, so healing a party member through their frame hit the
    // current target.
    uint64_t target = 0;
    if (const char* unit = luaL_optstring(L, 2, nullptr)) {
        std::string u(unit);
        toLowerInPlace(u);
        target = resolveUnitGuid(gh, u);
    }
    if (target == 0 && gh->hasTarget()) target = gh->getTargetGuid();
    gh->castSpell(spellId, target);
    return 0;
}

static int lua_IsUsableSpell(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { lua_pushboolean(L, 0); lua_pushboolean(L, 0); return 2; }

    uint32_t spellId = 0;
    if (lua_isnumber(L, 1)) {
        spellId = static_cast<uint32_t>(lua_tonumber(L, 1));
    } else if (lua_isstring(L, 1)) {
        const char* name = lua_tostring(L, 1);
        if (!name || !*name) { lua_pushboolean(L, 0); lua_pushboolean(L, 0); return 2; }
        std::string nameLow(name);
        toLowerInPlace(nameLow);
        for (uint32_t sid : gh->getKnownSpells()) {
            std::string sn = gh->getSpellName(sid);
            toLowerInPlace(sn);
            if (sn == nameLow) { spellId = sid; break; }
        }
    }

    if (spellId == 0 || !gh->getKnownSpells().count(spellId)) {
        lua_pushboolean(L, 0);
        lua_pushboolean(L, 0);
        return 2;
    }

    float cd = gh->getSpellCooldown(spellId);
    bool onCooldown = (cd > 0.1f);
    bool noMana = false;
    if (!onCooldown) {
        auto spellData = gh->getSpellData(spellId);
        if (spellData.manaCost > 0) {
            auto playerEntity = gh->getEntityManager().getEntity(gh->getPlayerGuid());
            if (playerEntity) {
                auto* unit = dynamic_cast<game::Unit*>(playerEntity.get());
                if (unit && unit->getPower() < spellData.manaCost) {
                    noMana = true;
                }
            }
        }
    }
    lua_pushboolean(L, (onCooldown || noMana) ? 0 : 1);
    lua_pushboolean(L, noMana ? 1 : 0);
    return 2;
}


void registerSpellLuaAPI(lua_State* L) {
    static const struct { const char* name; lua_CFunction func; } api[] = {
                {"GetTotemInfo",     lua_GetTotemInfo},
                // DestroyTotem(slot) - pull one down early. The totem bar's
                // right-click, and the slot is all the request carries.
                {"DestroyTotem", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int slot = static_cast<int>(luaL_optnumber(L, 1, 0));
            // The bar counts from one and the request from zero.
            if (gh && slot >= 1) gh->destroyTotem(slot - 1);
            return 0;
        }},
                {"GetTotemTimeLeft", lua_GetTotemTimeLeft},
                {"SpellStopCasting", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (gh) gh->cancelCast();
            return 0;
        }},
                {"SpellStopTargeting", [](lua_State* L) -> int {
            // No AoE reticle, but the item cursor a stone or an oil arms is a
            // kind of targeting and escape has to put it down.
            auto* gh = getGameHandler(L);
            if (gh && gh->isAwaitingItemTarget()) gh->cancelItemTargeting();
            return 0;
        }},
                {"SpellIsTargeting", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushboolean(L, gh && gh->isAwaitingItemTarget());
            return 1;
        }},
                {"IsSpellInRange",    lua_IsSpellInRange},
                {"UnitBuff",          lua_UnitBuff},
                {"UnitDebuff",        lua_UnitDebuff},
                {"UnitAura",          lua_UnitAuraGeneric},
                {"UnitCastingInfo",   lua_UnitCastingInfo},
                {"UnitChannelInfo",   lua_UnitChannelInfo},
                {"CastSpellByName",   lua_CastSpellByName},
                {"CastSpellByID",       lua_CastSpellByID},
                {"IsSpellKnown",      lua_IsSpellKnown},
                {"GetNumSpellTabs",   lua_GetNumSpellTabs},
                {"GetSpellTabInfo",   lua_GetSpellTabInfo},
                {"GetSpellBookItemInfo", lua_GetSpellBookItemInfo},
                {"GetSpellBookItemName", lua_GetSpellBookItemName},
                {"GetSpellName",      lua_GetSpellName},
                {"SpellTargetUnit",   lua_SpellTargetUnit},
                {"SpellTargetItem",   lua_SpellTargetItem},
                {"DropItemOnUnit",    lua_DropItemOnUnit},
                {"SetMultiCastSpell", lua_SetMultiCastSpell},
                {"CastSpell",         lua_CastSpell},
                {"IsPassiveSpell",    lua_IsPassiveSpell},
                {"IsSelectedSpell",   lua_IsSelectedSpell},
                {"GetSpellAutocast",  lua_GetSpellAutocast},
                {"GetKnownSlotFromHighestRankSlot", lua_GetKnownSlotFromHighestRankSlot},
                {"UpdateSpells",      lua_UpdateSpells},
                {"GetSpellCooldown",  lua_GetSpellCooldown},
                {"GetSpellPowerCost", lua_GetSpellPowerCost},
                {"GetSpellDescription", lua_GetSpellDescription},
                {"GetEnchantInfo",     lua_GetEnchantInfo},
                {"GetSpellInfo",      lua_GetSpellInfo},
                {"GetSpellTexture",   lua_GetSpellTexture},
                {"GetSpellLink",         lua_GetSpellLink},
                {"IsUsableSpell",        lua_IsUsableSpell},
                {"CancelUnitBuff",      lua_CancelUnitBuff},
                {"HasTarget",         lua_HasTarget},
                {"TargetUnit",        lua_TargetUnit},
                {"ClearTarget",       lua_ClearTarget},
                {"FocusUnit",         lua_FocusUnit},
                {"ClearFocus",        lua_ClearFocus},
                {"AssistUnit",        lua_AssistUnit},
                {"TargetLastTarget",  lua_TargetLastTarget},
                {"TargetNearestEnemy",  lua_TargetNearestEnemy},
                {"TargetNearestFriend", lua_TargetNearestFriend},
                {"GetRaidTargetIndex",  lua_GetRaidTargetIndex},
                {"SetRaidTarget",       lua_SetRaidTarget},
                {"IsPlayerSpell", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            uint32_t spellId = static_cast<uint32_t>(luaL_checknumber(L, 1));
            lua_pushboolean(L, gh && gh->getKnownSpells().count(spellId) ? 1 : 0);
            return 1;
        }},
                {"IsSpellOverlayed", [](lua_State* L) -> int {
            (void)L; lua_pushboolean(L, 0); return 1; // No proc overlay tracking
        }},
                {"IsCurrentSpell", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            uint32_t spellId = static_cast<uint32_t>(luaL_checknumber(L, 1));
            lua_pushboolean(L, gh && gh->getCurrentCastSpellId() == spellId ? 1 : 0);
            return 1;
        }},
                {"IsAutoRepeatSpell", [](lua_State* L) -> int {
            (void)L; lua_pushboolean(L, 0); return 1; // Stub
        }},
                {"CastShapeshiftForm", [](lua_State* L) -> int {
            // CastShapeshiftForm(index) - cast the spell for the given form slot
            auto* gh = getGameHandler(L);
            int index = static_cast<int>(luaL_checknumber(L, 1));
            if (!gh || index < 1) return 0;
            uint8_t classId = gh->getPlayerClass();
            const auto forms = game::knownShapeshiftForms(classId, gh->getKnownSpells());
            if (static_cast<size_t>(index) > forms.size()) return 0;
            gh->castSpell(forms[static_cast<size_t>(index) - 1].spellId, 0);
            return 0;
        }},
                {"CancelShapeshiftForm", [](lua_State* L) -> int {
            // Cancel current form - cast spell 0 or cancel aura
            auto* gh = getGameHandler(L);
            if (gh && gh->getShapeshiftFormId() != 0) {
                // Cancelling a form is done by re-casting the same form spell
                // For simplicity, just note that the server will handle it
            }
            return 0;
        }},
                // GetShapeshiftFormCooldown is defined in the bootstrap, over
                // GetShapeshiftFormInfo and GetSpellCooldown, because those are
                // in two different files and it needs both. A stub here would
                // be dead - the bootstrap runs after these are registered - and
                // framexml_lua_override_check exists to catch exactly that.
                {"GetShapeshiftForm", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushnumber(L, gh ? gh->getShapeshiftFormId() : 0);
            return 1;
        }},
                {"GetNumShapeshiftForms", [](lua_State* L) -> int {
            // Return count based on player class
            auto* gh = getGameHandler(L);
            if (!gh) { return luaReturnZero(L); }
            uint8_t classId = gh->getPlayerClass();
            // Only the forms this character has actually learned, from the
            // one table GetShapeshiftFormInfo and CastShapeshiftForm also read.
            // The bar walks 1..this and asks the other two about each index, so
            // a count taken from a different list describes one form and casts
            // another.
            lua_pushnumber(L, static_cast<double>(
                game::knownShapeshiftForms(classId, gh->getKnownSpells()).size()));
            return 1;
        }},
    };
    for (const auto& [name, func] : api) {
        lua_pushcfunction(L, func);
        lua_setglobal(L, name);
    }
}

} // namespace wowee::addons
