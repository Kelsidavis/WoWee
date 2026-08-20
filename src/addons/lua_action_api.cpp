// lua_action_api.cpp - Action bar, cursor/pickup, keyboard input, key bindings, and pet actions Lua API bindings.
// Extracted from lua_engine.cpp as part of §5.1 (Tame LuaEngine).
#include "game/item_text.hpp"
#include "addons/lua_api_helpers.hpp"
// For fireEvent: paging has to reach the frames as well as the addons.
#include "addons/lua_engine.hpp"
#include "game/inventory_slots.hpp"
#include "ui/framexml_takeover.hpp"
#include "ui/keybinding_manager.hpp"
#include "core/config_paths.hpp"
#include <array>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <map>
#include "game/pet_action.hpp"
#include "imgui.h"
#include <optional>
#include <SDL2/SDL_keyboard.h>

namespace wowee::addons {

// What the interface is holding.
//
// This is the original interface's cursor, and it is not the only one: this
// client's inventory screen keeps a held item of its own. They do not need
// merging, because whichever interface is drawing the bags is the one the
// player can pick anything up from, and that is the cursor in use - with the
// bags owned, as they are by default, this is it.
//
// What that does mean is that CursorHasItem answers for this cursor alone. An
// addon asking while the player is dragging in this client's own bags is told
// no, which is true of the cursor it can see and not of the player's hand.
// MERCHANT is a fifth kind and it is not an item: what the cursor holds is a
// vendor's list index, not something the player owns. FrameXML's merchant
// window puts one there on every left-click - PickupMerchantItem - and buying
// is what happens when it is dropped into a bag, so without it a left-click at
// a vendor did nothing at all and only right-click bought.
enum class CursorType { NONE, SPELL, ITEM, ACTION, MACRO, MERCHANT, MONEY, GUILDBANK };
static CursorType s_cursorType = CursorType::NONE;
static uint32_t   s_cursorId   = 0;    // spellId, itemId, or action slot
static int        s_cursorSlot = 0;    // source slot for placement
/// Which container the cursor's item came out of, and one of the four values
/// game::slots documents: 0 the backpack, 1-4 a worn bag, -1 the paperdoll,
/// kCursorNoSource the action bar. See cursorSourceIsInventory.
static int        s_cursorBag  = -1;
static uint64_t   s_cursorMoney = 0;   // copper, when the cursor carries money
/// How much of a guild bank stack the cursor took, or zero for all of it. The
/// wire carries the split on the *withdrawal*, not on the pickup, so the amount
/// has to wait on the cursor until the drop says where it is going.
static uint32_t   s_cursorSplit = 0;
static void setCursorType(lua_State* L, CursorType type);
/// Where the cursor's item came from, in wire bag/slot space. Declared here
/// because the guild bank pickup above needs it and it is defined further down
/// with the other cursor helpers.
static bool cursorWireSlot(uint8_t& bag, uint8_t& slot);
/// Whether a drop on an inventory slot is a move at all, and puts the cursor
/// down when it is not. See kCursorNoSource: an item off the action bar has
/// nowhere to be moved from, so dropping it in a bag or on the paperdoll does
/// nothing but end the drag, which is what the real client does.
static bool droppedItemFromNowhere(lua_State* L);

uint64_t cursorMoney() {
    return s_cursorType == CursorType::MONEY ? s_cursorMoney : 0;
}

void setCursorMoney(lua_State* L, uint64_t copper) {
    // Through setCursorType rather than by assignment: it is what fires
    // CURSOR_UPDATE and what shows the action bars' empty slots. Setting the
    // type by hand would put money on the cursor with nothing told about it.
    s_cursorMoney = copper;
    if (copper == 0) {
        if (s_cursorType == CursorType::MONEY) setCursorType(L, CursorType::NONE);
        return;
    }
    s_cursorId = 0;
    s_cursorSlot = 0;
    s_cursorBag = -1;
    setCursorType(L, CursorType::MONEY);
}

uint32_t cursorItemId() {
    return s_cursorType == CursorType::ITEM ? s_cursorId : 0;
}

int cursorEquipSlot() {
    // s_cursorBag is -1 only for a pickup off the paperdoll, where s_cursorSlot
    // is the one-based slot the item came out of.
    return (s_cursorType == CursorType::ITEM && s_cursorBag == -1) ? s_cursorSlot : 0;
}

/// PickupPetAction(slot) - pick a pet ability up off the pet bar.
///
/// Defined and does nothing. The cursor here holds spells, items, actions and
/// macros; a pet action is a fifth kind in its own slot space, and moving one
/// is a server operation this client does not send. Inventing a cursor state
/// that cannot be put down anywhere would be worse than not picking up.
///
/// Defined because it is reachable three ways - shift-clicking a pet button,
/// dragging one, and dropping on one - and the pet bar is drawn. Undefined,
/// the first shift-click on a pet ability takes the bar down. Clicking a pet
/// ability to use it goes through a different path and works; only rearranging
/// the bar is lost.
static int lua_PickupPetAction(lua_State* L) { (void)L; return 0; }

/// Change what the cursor is holding, and say so.
///
/// CURSOR_UPDATE is how the interface learns the cursor picked something up or
/// put it down - action buttons redraw their highlight on it, and the
/// equipment and container frames test the cursor from it. Thirteen places set
/// this and none of them said anything, so the cursor changed silently.
///
/// Every one of those thirteen goes through here. Converting some and not
/// others would give a cursor that announces half its changes, which is harder
/// to reason about than one that announces none.
///
/// Only on an actual change: several of these run on every click of an empty
/// slot, setting NONE over NONE, and the interface would redraw for each.
/// Defined below, beside the rest of the cursor state it clears. Declared here
/// because the sites that put the cursor down come first in the file, and each
/// of them used to clear only part of what a pickup sets.
static void clearCursorItem(lua_State* L);

static void setCursorType(lua_State* L, CursorType type) {
    if (s_cursorType == type) return;
    // Putting anything down forgets the guild bank split. It rides on the
    // cursor between the pickup and the drop, and a leftover would be applied
    // to the next withdrawal - a stack the player never asked to divide.
    if (type != CursorType::GUILDBANK) s_cursorSplit = 0;
    const bool wasCarrying = (s_cursorType != CursorType::NONE);
    const bool nowCarrying = (type != CursorType::NONE);
    s_cursorType = type;
    lua_getfield(L, LUA_REGISTRYINDEX, "wowee_lua_engine");
    auto* engine = static_cast<LuaEngine*>(lua_touserdata(L, -1));
    lua_pop(L, 1);
    if (!engine) return;
    engine->fireEvent("CURSOR_UPDATE", {});
    // Picking something up shows the empty slots of the action bars, so there
    // is somewhere visible to drop it; putting it down hides them again. Every
    // action button registers for both and neither was ever fired, so dragging
    // a spell towards the bar showed no targets at all - the bar looked as
    // though it would not take it.
    //
    // Fired on the change rather than on every set: this function is called
    // with the same type repeatedly and the grid would flicker.
    if (nowCarrying && !wasCarrying) {
        engine->fireEvent("ACTIONBAR_SHOWGRID", {});
    } else if (wasCarrying && !nowCarrying) {
        engine->fireEvent("ACTIONBAR_HIDEGRID", {});
    }
}

static int lua_HasAction(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { return luaReturnFalse(L); }
    int slot = static_cast<int>(luaL_checknumber(L, 1)) - 1; // WoW uses 1-indexed slots
    const auto& bar = gh->getActionBar();
    if (slot < 0 || slot >= static_cast<int>(bar.size())) {
        lua_pushboolean(L, 0);
        return 1;
    }
    lua_pushboolean(L, !bar[slot].isEmpty());
    return 1;
}

// GetActionTexture(slot) → texturePath or nil
static int lua_GetActionTexture(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { return luaReturnNil(L); }
    int slot = static_cast<int>(luaL_checknumber(L, 1)) - 1;
    const auto& bar = gh->getActionBar();
    if (slot < 0 || slot >= static_cast<int>(bar.size()) || bar[slot].isEmpty()) {
        lua_pushnil(L);
        return 1;
    }
    const auto& action = bar[slot];
    if (action.type == game::ActionBarSlot::SPELL) {
        std::string icon = gh->getSpellIconPath(action.id);
        if (!icon.empty()) {
            lua_pushstring(L, icon.c_str());
            return 1;
        }
    } else if (action.type == game::ActionBarSlot::ITEM && action.id != 0) {
        const auto* info = gh->getItemInfo(action.id);
        if (info && info->displayInfoId != 0) {
            std::string icon = gh->getItemIconPath(info->displayInfoId);
            if (!icon.empty()) {
                lua_pushstring(L, icon.c_str());
                return 1;
            }
        }
    }
    lua_pushnil(L);
    return 1;
}

// IsCurrentAction(slot) → boolean
static int lua_IsCurrentAction(lua_State* L) {
    // Currently no "active action" tracking; return false
    (void)L;
    lua_pushboolean(L, 0);
    return 1;
}

// IsUsableAction(slot) → usable, notEnoughMana
static int lua_IsUsableAction(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { lua_pushboolean(L, 0); lua_pushboolean(L, 0); return 2; }
    int slot = static_cast<int>(luaL_checknumber(L, 1)) - 1;
    const auto& bar = gh->getActionBar();
    if (slot < 0 || slot >= static_cast<int>(bar.size()) || bar[slot].isEmpty()) {
        lua_pushboolean(L, 0);
        lua_pushboolean(L, 0);
        return 2;
    }
    const auto& action = bar[slot];
    // Not the cooldown. WoW's answer here is about whether the action *could*
    // be used - known, affordable, the right stance - and a spell waiting on a
    // cooldown is all of those. The wait is shown by the sweep over the icon,
    // which is drawn from GetActionCooldown and nothing to do with this.
    //
    // Folding it in here painted the icon dark grey for the whole of every
    // cooldown, because ActionButton_UpdateUsable has three branches and
    // "not usable, mana is fine" is the one that means unusable.
    bool usable = true;
    bool noMana = false;
    if (action.type == game::ActionBarSlot::SPELL) {
        usable = gh->getKnownSpells().count(action.id) > 0;
        // Check power cost
        if (usable && action.id != 0) {
            auto spellData = gh->getSpellData(action.id);
            if (spellData.manaCost > 0) {
                auto pe = gh->getEntityManager().getEntity(gh->getPlayerGuid());
                if (pe) {
                    auto* unit = dynamic_cast<game::Unit*>(pe.get());
                    if (unit && unit->getPower() < spellData.manaCost) {
                        noMana = true;
                        usable = false;
                    }
                }
            }
        }
    }
    lua_pushboolean(L, usable ? 1 : 0);
    lua_pushboolean(L, noMana ? 1 : 0);
    return 2;
}

// IsActionInRange(slot) → 1 if in range, 0 if out, nil if no range check applicable
static int lua_IsActionInRange(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { return luaReturnNil(L); }
    int slot = static_cast<int>(luaL_checknumber(L, 1)) - 1;
    const auto& bar = gh->getActionBar();
    if (slot < 0 || slot >= static_cast<int>(bar.size()) || bar[slot].isEmpty()) {
        lua_pushnil(L);
        return 1;
    }
    const auto& action = bar[slot];
    uint32_t spellId = 0;
    if (action.type == game::ActionBarSlot::SPELL) {
        spellId = action.id;
    } else {
        // Items/macros: no range check for now
        lua_pushnil(L);
        return 1;
    }
    if (spellId == 0) { return luaReturnNil(L); }

    auto data = gh->getSpellData(spellId);
    if (data.maxRange <= 0.0f) {
        // Melee or self-cast spells: no range indicator
        lua_pushnil(L);
        return 1;
    }

    // Need a target to check range against
    uint64_t targetGuid = gh->getTargetGuid();
    if (targetGuid == 0) { return luaReturnNil(L); }
    auto targetEnt = gh->getEntityManager().getEntity(targetGuid);
    auto playerEnt = gh->getEntityManager().getEntity(gh->getPlayerGuid());
    if (!targetEnt || !playerEnt) { return luaReturnNil(L); }

    float dx = playerEnt->getX() - targetEnt->getX();
    float dy = playerEnt->getY() - targetEnt->getY();
    float dz = playerEnt->getZ() - targetEnt->getZ();
    float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
    lua_pushnumber(L, dist <= data.maxRange ? 1 : 0);
    return 1;
}

// GetActionInfo(slot) → actionType, id, subType
static int lua_GetActionInfo(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { return 0; }
    int slot = static_cast<int>(luaL_checknumber(L, 1)) - 1;
    const auto& bar = gh->getActionBar();
    if (slot < 0 || slot >= static_cast<int>(bar.size()) || bar[slot].isEmpty()) {
        return 0;
    }
    const auto& action = bar[slot];
    switch (action.type) {
        case game::ActionBarSlot::SPELL:
            lua_pushstring(L, "spell");
            lua_pushnumber(L, action.id);
            lua_pushstring(L, "spell");
            // The fourth value, which for a spell action is the same id.
            // vehiclemenubar reads it to build a link on a modified click -
            // HandleModifiedItemClick(GetSpellLink(spellID)) - so without it
            // shift-clicking a vehicle button linked nothing. Only spells have
            // one; an item or a macro answers nil, as the real client does.
            lua_pushnumber(L, action.id);
            return 4;
        case game::ActionBarSlot::ITEM:
            lua_pushstring(L, "item");
            lua_pushnumber(L, action.id);
            lua_pushstring(L, "item");
            return 3;
        case game::ActionBarSlot::MACRO:
            lua_pushstring(L, "macro");
            lua_pushnumber(L, action.id);
            lua_pushstring(L, "macro");
            return 3;
        default:
            return 0;
    }
}

// GetActionCount(slot) → count (item stack count or 0)
static int lua_GetActionCount(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { return luaReturnZero(L); }
    int slot = static_cast<int>(luaL_checknumber(L, 1)) - 1;
    const auto& bar = gh->getActionBar();
    if (slot < 0 || slot >= static_cast<int>(bar.size()) || bar[slot].isEmpty()) {
        lua_pushnumber(L, 0);
        return 1;
    }
    const auto& action = bar[slot];
    if (action.type == game::ActionBarSlot::ITEM && action.id != 0) {
        lua_pushnumber(L, gh->getInventory().countItem(action.id));
    } else {
        lua_pushnumber(L, 0);
    }
    return 1;
}

// GetActionCooldown(slot) → start, duration, enable
static int lua_GetActionCooldown(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { lua_pushnumber(L, 0); lua_pushnumber(L, 0); lua_pushnumber(L, 1); return 3; }
    int slot = static_cast<int>(luaL_checknumber(L, 1)) - 1;
    const auto& bar = gh->getActionBar();
    if (slot < 0 || slot >= static_cast<int>(bar.size()) || bar[slot].isEmpty()) {
        lua_pushnumber(L, 0);
        lua_pushnumber(L, 0);
        lua_pushnumber(L, 1);
        return 3;
    }
    const auto& action = bar[slot];
    if (action.cooldownRemaining > 0.0f) {
        // WoW returns GetTime()-based start time; approximate
        double now = 0;
        lua_getglobal(L, "GetTime");
        if (lua_isfunction(L, -1)) {
            lua_call(L, 0, 1);
            now = lua_tonumber(L, -1);
            lua_pop(L, 1);
        } else {
            lua_pop(L, 1);
        }
        double start = now - (action.cooldownTotal - action.cooldownRemaining);
        lua_pushnumber(L, start);
        lua_pushnumber(L, action.cooldownTotal);
        lua_pushnumber(L, 1);
    } else if (action.type == game::ActionBarSlot::SPELL && gh->isGCDActive()) {
        // No individual cooldown but GCD is active - show GCD sweep
        float gcdRem = gh->getGCDRemaining();
        float gcdTotal = gh->getGCDTotal();
        double now = 0;
        lua_getglobal(L, "GetTime");
        if (lua_isfunction(L, -1)) { lua_call(L, 0, 1); now = lua_tonumber(L, -1); lua_pop(L, 1); }
        else lua_pop(L, 1);
        double elapsed = gcdTotal - gcdRem;
        lua_pushnumber(L, now - elapsed);
        lua_pushnumber(L, gcdTotal);
        lua_pushnumber(L, 1);
    } else {
        lua_pushnumber(L, 0);
        lua_pushnumber(L, 0);
        lua_pushnumber(L, 1);
    }
    return 3;
}

// UseAction(slot, unit, button) - activate action bar slot (1-indexed)
//
// The second argument is a unit, and it was ignored. securetemplates is the
// only caller and it passes what the button's own attributes resolved to:
// nil for an ordinary click, and "player" when the self-cast modifier is
// held - which is the whole of how self-casting works. Dropping it meant
// every action went at the current target, so self-cast did nothing, and a
// unit-frame button carrying its own unit acted on whoever was targeted
// instead.
static int lua_UseAction(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) return 0;
    int slot = static_cast<int>(luaL_checknumber(L, 1)) - 1;
    const auto& bar = gh->getActionBar();
    if (slot < 0 || slot >= static_cast<int>(bar.size()) || bar[slot].isEmpty()) return 0;
    const auto& action = bar[slot];
    // The end of the chain a click travels, and the last place it can stop
    // without saying so: an empty slot, a cooldown, or a type nothing acts on
    // all look the same from the button. At debug level, because pressing a
    // button on the action bar is the most ordinary thing a player does and
    // this was a warning on every one of them.
    LOG_DEBUG("UseAction: slot ", slot + 1, " type=", static_cast<int>(action.type),
              " id=", action.id, " ready=", action.isReady() ? 1 : 0);
    if (action.type == game::ActionBarSlot::SPELL && action.isReady()) {
        // A unit that resolves wins; anything else falls back to the target,
        // which is what an ordinary click has always done. An unresolvable
        // unit is not treated as "cast on nobody" - a raid frame naming a
        // member who has gone out of range should still cast at the target
        // rather than silently at nothing.
        uint64_t target = gh->hasTarget() ? gh->getTargetGuid() : 0;
        if (const char* unit = lua_tostring(L, 2); unit && *unit) {
            std::string uid(unit);
            toLowerInPlace(uid);
            if (const uint64_t named = resolveUnitGuid(gh, uid)) target = named;
        }
        // A spell that belongs to an item - a Hearthstone is spell 8690 - has
        // to go as CMSG_USE_ITEM rather than a cast, or the server drops it.
        // This client's own action bar did that and was the only place that
        // did; the bar is FrameXML's now and this is the path its buttons take.
        if (const uint32_t itemForSpell = gh->getItemIdForSpell(action.id)) {
            gh->useItemById(itemForSpell, target);
        } else {
            gh->castSpell(action.id, target);
        }
    } else if (action.type == game::ActionBarSlot::ITEM && action.id != 0) {
        gh->useItemById(action.id);
    }
    // Macro execution requires GameScreen context; not available from pure Lua API
    return 0;
}

// --- Cursor / Drag-Drop System ---
// Tracks what the player is "holding" on the cursor (spell, item, action).


static int lua_ClearCursor(lua_State* L) {
    // Everything a pickup set, including the icon drawn on the pointer.
    //
    // This cleared the type, the id, the slot and the bag and left the icon,
    // which is a separate piece of state that only clearCursorItem touches. So
    // cancelling a pickup - Escape, a right-click, a click on nothing, all of
    // which reach this - put the item back and left its picture stuck to the
    // cursor for the rest of the session.
    clearCursorItem(L);
    return 0;
}

static int lua_GetCursorInfo(lua_State* L) {
    switch (s_cursorType) {
        case CursorType::SPELL:
            lua_pushstring(L, "spell");
            lua_pushnumber(L, 0);          // bookSlotIndex
            lua_pushstring(L, "spell");    // bookType
            lua_pushnumber(L, s_cursorId); // spellId
            return 4;
        case CursorType::ITEM:
        // An item out of the guild bank is an item. Where it came from is this
        // file's business - PickupContainerItem reads the cursor type to know a
        // drop is a withdrawal - and saying so here would only give the
        // interface a kind it has no branch for.
        case CursorType::GUILDBANK:
            lua_pushstring(L, "item");
            lua_pushnumber(L, s_cursorId);
            return 2;
        case CursorType::ACTION:
            lua_pushstring(L, "action");
            lua_pushnumber(L, s_cursorSlot);
            return 2;
        case CursorType::MACRO:
            lua_pushstring(L, "macro");
            lua_pushnumber(L, s_cursorId);
            return 2;
        case CursorType::MONEY:
            // containerframe.lua branches on this exact string to decide that
            // a click in a bag is money being put back.
            lua_pushstring(L, "money");
            lua_pushnumber(L, static_cast<lua_Number>(s_cursorMoney));
            return 2;
        case CursorType::MERCHANT:
            // containerframe.lua branches on this exact string to decide
            // whether a click in a bag is a purchase.
            lua_pushstring(L, "merchant");
            lua_pushnumber(L, s_cursorId);   // the vendor's list index
            return 2;
        default:
            return 0;
    }
}

static int lua_CursorHasItem(lua_State* L) {
    // A guild bank item counts. The bank frame's own OnClick tests this before
    // deciding a click is a drop, so answering no for one would have left the
    // item stuck on the cursor with nowhere that would take it.
    lua_pushboolean(L, (s_cursorType == CursorType::ITEM ||
                        s_cursorType == CursorType::GUILDBANK) ? 1 : 0);
    return 1;
}

static int lua_CursorHasSpell(lua_State* L) {
    lua_pushboolean(L, s_cursorType == CursorType::SPELL ? 1 : 0);
    return 1;
}

// PickupAction(slot) - picks up an action from the action bar
/// PickupAction(slot) - take what is in a slot onto the cursor, and put down
/// what the cursor was already holding.
///
/// A swap, which is what the real client does and what the interface expects:
/// ActionButton_OnClick calls this for a click on a button whether or not
/// anything is held.
///
/// The slot used to be left exactly as it was. Picking an action up put a copy
/// on the cursor and the button went on showing it, so dragging something off
/// the bar looked like it had done nothing at all - and because the slot never
/// changed, no ACTIONBAR_SLOT_CHANGED was fired and nothing redrew. The empty
/// half only understood a spell, so an item or a macro dropped on a free slot
/// vanished.
static int lua_PickupAction(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) return 0;
    int slot = static_cast<int>(luaL_checknumber(L, 1));
    const auto& bar = gh->getActionBar();
    if (slot < 1 || slot > static_cast<int>(bar.size())) return 0;

    // Read before anything is written: setActionBarSlot rewrites the entry
    // this would otherwise still be pointing at.
    const auto existing = bar[slot - 1];
    const bool hadAction = !existing.isEmpty();
    const CursorType held = s_cursorType;
    const uint32_t heldId = s_cursorId;

    const auto typeForCursor = [](CursorType c) {
        return c == CursorType::SPELL ? game::ActionBarSlot::SPELL
             : c == CursorType::ITEM  ? game::ActionBarSlot::ITEM
                                      : game::ActionBarSlot::MACRO;
    };
    const bool holding = heldId != 0 &&
                         (held == CursorType::SPELL || held == CursorType::ITEM ||
                          held == CursorType::MACRO);

    if (holding) {
        gh->setActionBarSlot(slot - 1, typeForCursor(held), heldId);
    } else if (hadAction) {
        // Nothing held, so the slot empties. This is the fire that redraws the
        // button: setActionBarSlot raises ACTIONBAR_SLOT_CHANGED for it.
        gh->setActionBarSlot(slot - 1, game::ActionBarSlot::EMPTY, 0);
    }

    if (hadAction) {
        // What was there goes to the cursor, whether this was a swap or a
        // plain pick-up.
        setCursorType(L, (existing.type == game::ActionBarSlot::SPELL) ? CursorType::SPELL :
                       (existing.type == game::ActionBarSlot::ITEM)  ? CursorType::ITEM :
                       CursorType::ACTION);
        s_cursorId = existing.id;
        s_cursorSlot = slot;
        // Which button, not which slot of which bag - a fourth numbering, and
        // the reason the source has to say so. An item action carries a real
        // itemId, so without this the cursor was indistinguishable from one
        // lifted off the paperdoll and the button number was spent as an
        // equipment slot. See kCursorNoSource.
        s_cursorBag = game::slots::kCursorNoSource;
        // The shared cursor as well, which is a separate record of the same
        // thing and would otherwise still be naming wherever the *previous*
        // pickup came from: clicking a button while carrying a bag item swaps
        // the two, and the bag slot it left behind stayed on this cursor as
        // the source PutItemInBag would have moved from.
        cursorItemSlot() = {};
        // And on the pointer, or the drag reads as not having started.
        if (existing.type == game::ActionBarSlot::SPELL) {
            wowee::ui::frameXmlSetCursorItem(gh->getSpellIconPath(existing.id));
        } else if (existing.type == game::ActionBarSlot::ITEM) {
            const auto* info = gh->getItemInfo(existing.id);
            wowee::ui::frameXmlSetCursorItem(
                info ? gh->getItemIconPath(info->displayInfoId) : std::string());
        } else {
            wowee::ui::frameXmlSetCursorItem(std::string());
        }
    } else {
        clearCursorItem(L);
    }
    return 0;
}

// PlaceAction(slot) - places cursor content into an action bar slot
static int lua_PlaceAction(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) return 0;
    int slot = static_cast<int>(luaL_checknumber(L, 1));
    if (slot < 1 || slot > static_cast<int>(gh->getActionBar().size())) return 0;
    if (s_cursorType == CursorType::SPELL && s_cursorId != 0) {
        gh->setActionBarSlot(slot - 1, game::ActionBarSlot::SPELL, s_cursorId);
    } else if (s_cursorType == CursorType::ITEM && s_cursorId != 0) {
        gh->setActionBarSlot(slot - 1, game::ActionBarSlot::ITEM, s_cursorId);
    } else if (s_cursorType == CursorType::MACRO && s_cursorId != 0) {
        gh->setActionBarSlot(slot - 1, game::ActionBarSlot::MACRO, s_cursorId);
    }
    clearCursorItem(L);
    return 0;
}

// PickupSpell(bookSlot, bookType) - picks up a spell from the spellbook
static int lua_PickupSpell(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) return 0;
    // Through the shared resolver, so the book is read as well as the slot:
    // dragging a pet spell out of the book picked up whichever of the player's
    // spells sat at that index.
    const uint32_t picked = spellIdForCall(L, gh);
    if (picked == 0) return 0;
    setCursorType(L, CursorType::SPELL);
    s_cursorId = picked;
    // On the pointer as well as in the state, the way every other pickup here
    // does it. Dragging a spell out of the book carried nothing visible, so
    // the drag read as not having started.
    wowee::ui::frameXmlSetCursorItem(gh->getSpellIconPath(s_cursorId));
    return 0;
}

// PlaceGlyphInSocket(socket) - put the glyph on the cursor into that socket.
//
// Registered here rather than with the other glyph calls because the glyph is
// on the cursor and the cursor lives in this file. There is no socketing opcode
// in 3.3.5: the glyph item is used with the socket written into the glyphIndex
// field CMSG_USE_ITEM already carries, and the server applies the item's spell
// to it. The comment this replaces said socketing "needs a packet this client
// does not send" - the packet was there, one field short of saying where.
static int lua_PlaceGlyphInSocket(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int socket = static_cast<int>(luaL_optnumber(L, 1, 0));
    if (!gh || socket < 1 || socket > game::GameHandler::MAX_GLYPH_SLOTS) return 0;
    uint8_t bag = 0, slot = 0;
    if (!cursorWireSlot(bag, slot)) return 0;   // nothing held: nothing to place
    gh->placeGlyphFromBag(bag, slot, static_cast<uint32_t>(socket - 1));
    setCursorType(L, CursorType::NONE);
    return 0;
}

// PickupGuildBankItem(tab, slot) - both halves of a guild bank drag.
//
// The frame calls this from OnDragStart and from OnReceiveDrag, so one function
// picks up and puts down, the way PickupContainerItem does for bags. It was a
// no-op, so nothing could be moved into or out of the guild bank at all -
// while the packet builder for the move sat verified against the server, and
// AutoStoreGuildBankItem beside it has been using half of it for right-clicks.
//
// The tab and slot are FrameXML's, counted from one; the wire counts from zero,
// which is the shift AutoStoreGuildBankItem already applies.
static int lua_PickupGuildBankItem(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int tab = static_cast<int>(luaL_optnumber(L, 1, 0));
    const int slot = static_cast<int>(luaL_optnumber(L, 2, 0));
    if (!gh || tab < 1 || slot < 1) return 0;

    if (droppedItemFromNowhere(L)) return 0;

    // Carrying a bag item, so this is the deposit.
    uint8_t srcBag = 0, srcSlot = 0;
    if (cursorWireSlot(srcBag, srcSlot)) {
        gh->guildBankDepositItem(static_cast<uint8_t>(tab - 1),
                                 static_cast<uint8_t>(slot - 1), srcBag, srcSlot);
        setCursorType(L, CursorType::NONE);
        return 0;
    }
    // Carrying a bank item already: that is a move within the bank, and the
    // request builder says plainly that it does not write that path. Dropping
    // the cursor is better than sending something the server will read as a
    // withdrawal to nowhere.
    if (s_cursorType == CursorType::GUILDBANK) {
        setCursorType(L, CursorType::NONE);
        return 0;
    }

    // Otherwise pick the slot up, if there is anything in it.
    const auto& bank = gh->getGuildBankData();
    for (const auto& it : bank.tabItems) {
        if (it.slotId != slot - 1 || it.itemEntry == 0) continue;
        setCursorType(L, CursorType::GUILDBANK);
        s_cursorId = it.itemEntry;
        s_cursorBag = tab;
        s_cursorSlot = slot;
        s_cursorSplit = 0;   // the whole stack, unless SplitGuildBankItem says otherwise
        const auto* info = gh->getItemInfo(it.itemEntry);
        wowee::ui::frameXmlSetCursorItem(
            info ? gh->getItemIconPath(info->displayInfoId) : std::string());
        return 0;
    }
    return 0;
}

// SplitGuildBankItem(tab, slot, amount) - take part of a stack.
//
// The stack split dialog calls this instead of PickupGuildBankItem when the
// player asks for fewer than all of them. It is the same pickup with a number
// attached: the wire carries the split on the *withdrawal* rather than on the
// pickup, so the amount rides on the cursor until the drop says where it goes.
static int lua_SplitGuildBankItem(lua_State* L) {
    const int amount = static_cast<int>(luaL_optnumber(L, 3, 0));
    lua_settop(L, 2);
    lua_PickupGuildBankItem(L);
    if (s_cursorType == CursorType::GUILDBANK && amount > 0)
        s_cursorSplit = static_cast<uint32_t>(amount);
    return 0;
}

// PickupCompanion(mode, index) - drag a mount or a critter to the bar.
//
// Registered here rather than beside the other companion calls because the
// cursor lives in this file, and because that is what this is: a pickup.
//
// A companion is summoned by a spell, and this client's action bar holds
// spells, so the cursor carries the summon rather than a companion of its own
// kind. What lands on the bar is the same icon casting the same thing; the
// difference is invisible to everyone but the tooltip, which names the spell.
// Modelling a fourth action type to say "companion" would change nothing a
// player can see.
static int lua_PickupCompanion(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) return 0;
    const char* mode = luaL_optstring(L, 1, "");
    const int index = static_cast<int>(luaL_optnumber(L, 2, 0));
    if (index < 1) return 0;
    const auto& list = gh->getCompanions(std::string(mode) == "MOUNT");
    if (index > static_cast<int>(list.size())) return 0;
    const uint32_t spellId = list[static_cast<size_t>(index) - 1].spellId;
    if (spellId == 0) return 0;
    setCursorType(L, CursorType::SPELL);
    s_cursorId = spellId;
    wowee::ui::frameXmlSetCursorItem(gh->getSpellIconPath(spellId));
    return 0;
}

// PickupSpellBookItem(bookSlot, bookType) - alias for PickupSpell
static int lua_PickupSpellBookItem(lua_State* L) {
    return lua_PickupSpell(L);
}

// PickupContainerItem(bag, slot) - picks up an item from a bag

/// Where the item on the cursor lives, in the numbering the server uses.
///
/// The server addresses one flat space: equipment is slots 0 to 22, the
/// backpack follows at 23, and a bag's contents are addressed by that bag's own
/// equipment slot together with an index inside it. FrameXML counts bags 0 to 4
/// with 1-based slots instead, so the two have to be translated - the same
/// translation this client's own bag window does before sending a swap.
static bool cursorWireSlot(uint8_t& bag, uint8_t& slot) {
    if (s_cursorType != CursorType::ITEM) return false;
    // Nowhere to move it from. This used to fall into the equipped branch
    // below, because everything negative was the paperdoll - see
    // kCursorNoSource for what that cost.
    if (!game::slots::cursorSourceIsInventory(s_cursorBag)) return false;
    if (s_cursorBag < 0) {                    // an equipped item
        bag = 0xFF;
        slot = static_cast<uint8_t>(s_cursorSlot - 1);
    } else if (s_cursorBag == 0) {            // the backpack
        bag = 0xFF;
        slot = static_cast<uint8_t>(game::slots::backpackWireSlot(s_cursorSlot - 1));
    } else {                                   // one of the four worn bags
        bag = static_cast<uint8_t>(game::slots::wornBagContainer(s_cursorBag - 1));
        slot = static_cast<uint8_t>(s_cursorSlot - 1);
    }
    return true;
}

/// Put the cursor down.
static void clearCursorItem(lua_State* L) {
    s_cursorMoney = 0;
    s_cursorSplit = 0;
    setCursorType(L, CursorType::NONE);
    s_cursorId = 0;
    s_cursorSlot = 0;
    s_cursorBag = -1;
    wowee::ui::frameXmlSetCursorItem(std::string());
    cursorItemSlot() = {};
}

static bool droppedItemFromNowhere(lua_State* L) {
    if (s_cursorType != CursorType::ITEM) return false;
    if (game::slots::cursorSourceIsInventory(s_cursorBag)) return false;
    clearCursorItem(L);
    return true;
}

void pickupMerchantItem(lua_State* L, int index) {
    if (index < 1) return;
    setCursorType(L, CursorType::MERCHANT);
    s_cursorId = static_cast<uint32_t>(index);
    s_cursorSlot = 0;
    s_cursorBag = -1;
    // Show it on the pointer, as a bag pickup does. Without this the cursor
    // holds something invisible and the click reads as having done nothing.
    if (auto* gh = getGameHandler(L)) {
        const auto& items = gh->getVendorItems().items;
        if (index <= static_cast<int>(items.size())) {
            ui::frameXmlSetCursorItem(
                gh->getItemIconPath(items[index - 1].displayInfoId));
        }
    }
}

bool boughtHeldMerchantItem(lua_State* L) {
    if (s_cursorType != CursorType::MERCHANT) return false;
    auto* gh = getGameHandler(L);
    const int index = static_cast<int>(s_cursorId);
    if (gh && index >= 1) {
        const auto& items = gh->getVendorItems().items;
        if (index <= static_cast<int>(items.size())) {
            const auto& vi = items[index - 1];
            gh->buyItem(gh->getVendorGuid(), vi.itemId, vi.slot, 1);
        }
    }
    clearCursorItem(L);
    return true;
}


/// Put the item in a bag slot on the cursor, with everything that goes with it:
/// the icon, the greyed slot, and the record of where it came from.
///
/// Split out of lua_PickupContainerItem so PickupItem can reach it. Writing the
/// cursor twice is how one of them would come to set the icon and the other not.
static void pickupFromContainerSlot(lua_State* L, game::GameHandler* gh,
                                    int bag, int slot) {
    const auto& inv = gh->getInventory();
    const game::ItemSlot* itemSlot = nullptr;
    if (bag == 0 && slot >= 1 && slot <= inv.getBackpackSize()) {
        itemSlot = &inv.getBackpackSlot(slot - 1);
    } else if (bag >= 1 && bag <= 4) {
        int bagSize = inv.getBagSize(bag - 1);
        if (slot >= 1 && slot <= bagSize) {
            itemSlot = &inv.getBagSlot(bag - 1, slot - 1);
        }
    }
    if (itemSlot && !itemSlot->empty()) {
        setCursorType(L, CursorType::ITEM);
        s_cursorId = itemSlot->item.itemId;
        s_cursorBag = bag;
        s_cursorSlot = slot;
        uint32_t displayId = itemSlot->item.displayInfoId;
        if (displayId == 0) {
            if (const auto* info = gh->getItemInfo(itemSlot->item.itemId)) {
                displayId = info->displayInfoId;
            }
        }
        wowee::ui::frameXmlSetCursorItem(
            displayId ? gh->getItemIconPath(displayId) : std::string());
        cursorItemSlot() = {bag, slot, false};
        // The slot draws greyed while its item is on the cursor, which is how a
        // bag shows that something has been picked up out of it.
        gh->fireAddonEvent("ITEM_LOCK_CHANGED",
                           {std::to_string(bag), std::to_string(slot)});
        LOG_WARNING("FrameXML pickup: bag ", bag, " slot ", slot,
                    " item ", itemSlot->item.itemId);
    } else {
        LOG_WARNING("FrameXML pickup: bag ", bag, " slot ", slot,
                    " - nothing there to pick up");
    }
}

void pickupSplitFromContainer(lua_State* L, game::GameHandler* gh,
                              int bag, int slot, int count) {
    pickupFromContainerSlot(L, gh, bag, slot);
    // Only if the pickup took: an empty slot leaves the cursor alone, and an
    // amount left on it would ride along with whatever is picked up next.
    if (s_cursorType == CursorType::ITEM && count > 0) {
        s_cursorSplit = static_cast<uint32_t>(count);
    }
}


/// PickupItem(id | "name" | link) - put an item on the cursor by naming it
/// rather than by pointing at a slot.
///
/// A macro's own way to pick something up, and the 'item' branch of
/// securehandlers.lua's cursor dispatcher - every other branch there was bound
/// and this one raised.
///
/// The first matching slot wins. WoW picks the first too, and a stack split
/// across two bags is one item as far as this is concerned.
static int lua_ReturnNothing2(lua_State* L) { (void)L; return 0; }

static int lua_PickupItem(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) return 0;

    uint32_t wantId = 0;
    std::string wantName;
    if (lua_isnumber(L, 1)) {
        wantId = static_cast<uint32_t>(lua_tonumber(L, 1));
    } else if (const char* text = lua_tostring(L, 1)) {
        // A link carries the id in it - "|Hitem:6948:0:..." - and is what a
        // shift-click puts in a macro, so it is the common case rather than
        // the exotic one.
        const std::string str(text);
        const size_t at = str.find("Hitem:");
        if (at != std::string::npos) {
            wantId = static_cast<uint32_t>(std::strtoul(str.c_str() + at + 6, nullptr, 10));
        } else {
            wantName = str;
        }
    }
    if (wantId == 0 && wantName.empty()) return 0;

    auto matches = [&](const game::ItemSlot& s) {
        if (s.empty()) return false;
        if (wantId != 0) return s.item.itemId == wantId;
        return s.item.name == wantName;
    };

    const auto& inv = gh->getInventory();
    for (int i = 0; i < inv.getBackpackSize(); ++i) {
        if (matches(inv.getBackpackSlot(i))) {
            pickupFromContainerSlot(L, gh, 0, i + 1);
            return 0;
        }
    }
    for (int b = 0; b < game::Inventory::NUM_BAG_SLOTS; ++b) {
        for (int sl = 0; sl < inv.getBagSize(b); ++sl) {
            if (matches(inv.getBagSlot(b, sl))) {
                pickupFromContainerSlot(L, gh, b + 1, sl + 1);
                return 0;
            }
        }
    }
    return 0;
}

static int lua_PickupContainerItem(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) return 0;
    int bag = static_cast<int>(luaL_checknumber(L, 1));
    int slot = static_cast<int>(luaL_checknumber(L, 2));

    // Same as the paperdoll: a click while something is waiting for an item to
    // be applied to is that choice, not a pickup.
    if (completedItemTarget(L, containerSlotGuid(gh, bag, slot))) return 0;

    // With the repair cursor up, a click on an item is the repair. The real
    // client consumes the click in C for the same reason, which is why
    // containerframe.lua has no branch of its own for it.
    if (repairedHeldItem(gh, containerSlotGuid(gh, bag, slot))) return 0;

    // A bag click while the cursor holds a vendor's item is the purchase.
    // containerframe.lua routes it here after checking GetCursorInfo.
    if (boughtHeldMerchantItem(L)) return 0;

    // Carrying something out of the guild bank, so this is the withdrawal.
    // The destination is this bag slot rather than the server's choice, which
    // is what dropping on a particular square means.
    if (s_cursorType == CursorType::GUILDBANK) {
        uint8_t dstBag, dstSlot;
        if (bag == 0) {
            dstBag = 0xFF;
            dstSlot = static_cast<uint8_t>(game::slots::backpackWireSlot(slot - 1));
        } else {
            dstBag = static_cast<uint8_t>(game::slots::wornBagContainer(bag - 1));
            dstSlot = static_cast<uint8_t>(slot - 1);
        }
        gh->guildBankWithdrawItem(static_cast<uint8_t>(s_cursorBag - 1),
                                  static_cast<uint8_t>(s_cursorSlot - 1),
                                  dstBag, dstSlot, s_cursorSplit);
        setCursorType(L, CursorType::NONE);
        return 0;
    }

    // Carrying something the bags cannot take: the drag ends here and nothing
    // moves. Before cursorWireSlot, which would otherwise answer no and let
    // this fall through to the pickup below - putting the bag's own item on a
    // cursor that is already holding one.
    if (droppedItemFromNowhere(L)) return 0;

    // Already carrying something, so this is the drop rather than the pickup.
    // One function does both halves of a drag in WoW, and without this half a
    // dragged item was picked up and never put down anywhere.
    uint8_t srcBag = 0, srcSlot = 0;
    if (cursorWireSlot(srcBag, srcSlot)) {
        uint8_t dstBag, dstSlot;
        if (bag == 0) {
            dstBag = 0xFF;
            dstSlot = static_cast<uint8_t>(game::slots::backpackWireSlot(slot - 1));
        } else {
            dstBag = static_cast<uint8_t>(game::slots::wornBagContainer(bag - 1));
            dstSlot = static_cast<uint8_t>(slot - 1);
        }
        // At warning level because it is the outcome of a drag and happens
        // once per drop, and because the log carries nothing below warning -
        // which is why "did the move go out" could not be answered at all.
        LOG_WARNING("FrameXML drop: bag ", s_cursorBag, " slot ", s_cursorSlot,
                    " (wire ", (int)srcBag, "/", (int)srcSlot, ") -> bag ", bag,
                    " slot ", slot, " (wire ", (int)dstBag, "/", (int)dstSlot, ")");
        // Back where it came from: put it down rather than asking the server to
        // swap a slot with itself.
        if (s_cursorBag == bag && s_cursorSlot == slot) {
            const int sameBag = bag, sameSlot = slot;
            clearCursorItem(L);
            gh->fireAddonEvent("ITEM_LOCK_CHANGED",
                               {std::to_string(sameBag), std::to_string(sameSlot)});
            return 0;
        }

        const int wasBag = s_cursorBag, wasSlot = s_cursorSlot;
        // Part of a stack moves as a split rather than a swap: a swap would
        // send the whole thing, which is not what was picked up.
        if (s_cursorSplit > 0) {
            gh->splitItemTo(srcBag, srcSlot, dstBag, dstSlot,
                            static_cast<uint8_t>(s_cursorSplit));
        } else {
            gh->swapContainerItems(srcBag, srcSlot, dstBag, dstSlot);
        }
        clearCursorItem(L);
        gh->fireAddonEvent("ITEM_LOCK_CHANGED",
                           {std::to_string(wasBag), std::to_string(wasSlot)});
        gh->fireAddonEvent("ITEM_LOCK_CHANGED",
                           {std::to_string(bag), std::to_string(slot)});
        return 0;
    }

    pickupFromContainerSlot(L, gh, bag, slot);
    return 0;
}

// PickupInventoryItem(slot) - picks up an equipped item
// ClickSendMailItemButton(index, isRightClick) - attach what is held to the
// letter, or take an attachment back.
//
// Lives here rather than with the other mail functions because the cursor
// does, and this is entirely about the cursor: carrying an item means attach
// it, carrying nothing means the slot clicked gives its item back to the bags.
// The attachment index is optional - the drop handler calls this with no
// arguments at all, meaning "attach to wherever there is room", which is what
// attachItemFrom* already does.
static int lua_ClickSendMailItemButton(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) return 0;

    if (s_cursorType == CursorType::ITEM) {
        // The cursor counts the backpack as bag zero and the worn bags from
        // one, with slots from one; the attach functions count both from zero.
        const bool attached = (s_cursorBag == 0)
            ? gh->attachItemFromBackpack(s_cursorSlot - 1)
            : (s_cursorBag > 0
                   ? gh->attachItemFromBag(s_cursorBag - 1, s_cursorSlot - 1)
                   : false);
        if (attached) clearCursorItem(L);
        return 0;
    }

    // Nothing held: the click takes an attachment off the letter. Without an
    // index there is nothing to take off, since the caller meant a drop.
    const int index = static_cast<int>(luaL_optnumber(L, 1, 0));
    if (index >= 1) gh->detachMailAttachment(index - 1);
    return 0;
}

// PickupBagFromSlot(inventorySlot) - pick up or put down a bank bag.
//
// The bank's bag buttons hand over an inventory slot rather than an index:
// BankButtonIDToInvSlotID puts the seven bank bags at sixty-eight upward, and
// the wire counts from zero, so the slot sent is one less. That is the same
// arithmetic PickupInventoryItem does for worn equipment, and the cursor
// carries it the same way - a negative bag means "not in a container", which
// cursorWireSlot turns back into container 0xFF and the slot less one.
//
// Deferred for a long time on the belief that this client had no cursor to
// drop things from. It has had one all along, with both halves of a drag in
// one function, and the wire numbers here are the ones its own bank window
// already sends.
/// The nth worn bag, as Lua numbers inventory slots - the four beside the
/// backpack. BagSlotButton_OnDrag passes exactly these.
static constexpr int kFirstWornBagInventorySlot =
    game::slots::toInventorySlot(game::slots::wornBagContainer(0));

static int lua_PickupBagFromSlot(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) return 0;
    const int slot = static_cast<int>(luaL_optnumber(L, 1, 0));

    const int bankIndex = slot - game::slots::kFirstBankBagInventorySlot;
    const int wornIndex = slot - kFirstWornBagInventorySlot;
    const bool isBank = bankIndex >= 0 && bankIndex < game::Inventory::BANK_BAG_SLOTS;
    const bool isWorn = wornIndex >= 0 && wornIndex < game::Inventory::NUM_BAG_SLOTS;
    // Only the bank half was handled, so dragging one of the four worn bags did
    // nothing at all - it returned here before doing anything. Latent until the
    // bag bar was handed over, because this client's own bar never called it.
    if (!isBank && !isWorn) return 0;

    if (droppedItemFromNowhere(L)) return 0;

    // Carrying something: this is the drop, into the bag slot.
    uint8_t srcBag = 0, srcSlot = 0;
    if (cursorWireSlot(srcBag, srcSlot)) {
        const int srcWorn = s_cursorSlot - kFirstWornBagInventorySlot;
        const bool fromWorn = s_cursorBag == -1 &&
                              srcWorn >= 0 && srcWorn < game::Inventory::NUM_BAG_SLOTS;
        if (fromWorn && isWorn) {
            // Both are worn bags, which has its own verb: it swaps the bags'
            // contents locally as well as sending the move, so the bags on
            // screen do not have to wait for the server to agree.
            gh->swapBagSlots(srcWorn, wornIndex);
        } else {
            gh->swapContainerItems(srcBag, srcSlot, 0xFF,
                                   static_cast<uint8_t>(game::slots::toWireSlot(slot)));
        }
        clearCursorItem(L);
        return 0;
    }

    const auto& held = isBank
        ? gh->getInventory().getBankBagItem(bankIndex)
        : gh->getInventory().getEquipSlot(static_cast<game::EquipSlot>(
              static_cast<int>(game::EquipSlot::BAG1) + wornIndex));
    if (held.empty()) {
        // Dragging a bag that is plainly in the slot has been reported doing
        // nothing, and every step of the arithmetic checks out on paper: the
        // button's id, the worn index, the equipment slot. So say what the
        // slot actually held when the drag arrived, rather than returning in
        // silence as this has always done for an empty slot.
        LOG_WARNING("PickupBagFromSlot: slot ", slot, " (worn index ", wornIndex,
                    ", bank index ", bankIndex, ") reads as empty - nothing to pick up");
        return 0;
    }
    setCursorType(L, CursorType::ITEM);
    s_cursorId = held.item.itemId;
    s_cursorSlot = slot;
    s_cursorBag = -1;
    // The shared cursor as well as this file's own. Everything that receives a
    // drop reads cursorItemSlot(), and a worn bag is an equipped item like any
    // other - PickupInventoryItem below sets exactly this. Without it the bag
    // lifted onto the pointer and then every drop target reported nothing
    // held: PutItemInBag returned false, so BagSlotButton_OnClick fell through
    // to ToggleBag and the bag opened instead of moving.
    cursorItemSlot() = {-1, slot, true};
    // Same omission as the spell above: a bag dragged off its slot was held
    // invisibly. The display id falls back to the item's own, which is where
    // the equipped-item pickup below reads it from when the slot has none.
    uint32_t displayId = held.item.displayInfoId;
    if (displayId == 0) {
        if (const auto* info = gh->getItemInfo(held.item.itemId)) {
            displayId = info->displayInfoId;
        }
    }
    wowee::ui::frameXmlSetCursorItem(
        displayId ? gh->getItemIconPath(displayId) : std::string());
    // And say the slot is locked, as the equipped pickup below does.
    // IsInventoryItemLocked already answers true for it - cursorEquipSlot
    // reads the same s_cursorSlot this just set - but nothing asks again
    // without this, so the square greyed only on the next unrelated redraw.
    // One argument, because the paperdoll's handler tests `not arg2`.
    gh->fireAddonEvent("ITEM_LOCK_CHANGED", {std::to_string(slot)});
    return 0;
}

static int lua_PickupInventoryItem(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) return 0;
    int slot = static_cast<int>(luaL_checknumber(L, 1));
    if (slot < 1 || slot > 23) return 0;

    // Before the pickup, because the paperdoll has no SpellCanTargetItem branch
    // of its own - a left-click on a worn weapon always arrives here, and while
    // a stone or an oil is waiting for a target that click is the target.
    if (slot <= 19 && completedItemTarget(L, gh->getEquipSlotGuid(slot - 1))) return 0;

    // Worn gear is what usually needs repairing, and the paperdoll has no
    // branch of its own for it either.
    if (slot <= 19 && repairedHeldItem(gh, gh->getEquipSlotGuid(slot - 1))) return 0;

    if (droppedItemFromNowhere(L)) return 0;

    // Carrying something: equip it here, which is the other half of the drag.
    uint8_t srcBag = 0, srcSlot = 0;
    if (cursorWireSlot(srcBag, srcSlot)) {
        gh->swapContainerItems(srcBag, srcSlot, 0xFF,
                               static_cast<uint8_t>(slot - 1));
        clearCursorItem(L);
        return 0;
    }

    const auto& inv = gh->getInventory();
    const auto& eq = inv.getEquipSlot(static_cast<game::EquipSlot>(slot - 1));
    if (!eq.empty()) {
        setCursorType(L, CursorType::ITEM);
        s_cursorId = eq.item.itemId;
        s_cursorSlot = slot;
        s_cursorBag = -1;
        // The same three things a container pickup does, which this had none
        // of: remember where it came from, put its icon on the pointer, and say
        // the slot is locked. Without them dragging off the character sheet
        // carried nothing visible and left the slot looking untouched.
        cursorItemSlot() = {-1, slot, true};
        uint32_t displayId = eq.item.displayInfoId;
        if (displayId == 0) {
            if (const auto* info = gh->getItemInfo(eq.item.itemId)) {
                displayId = info->displayInfoId;
            }
        }
        wowee::ui::frameXmlSetCursorItem(
            displayId ? gh->getItemIconPath(displayId) : std::string());
        // One argument, not two: the paperdoll's handler is
        // `if ( not arg2 and arg1 == self:GetID() )`, so an equipment lock is
        // the slot alone. A second argument makes that test fail and the square
        // never greys.
        gh->fireAddonEvent("ITEM_LOCK_CHANGED", {std::to_string(slot)});
        LOG_WARNING("FrameXML pickup: equipment slot ", slot,
                    " item ", eq.item.itemId);
    } else {
        LOG_WARNING("FrameXML pickup: equipment slot ", slot,
                    " - nothing equipped there");
    }
    return 0;
}

// DeleteCursorItem() - destroys the item on the cursor.
//
// It only put the cursor down. Nothing was ever destroyed: the item vanished
// from the cursor, the server was never told, and the next bag update put it
// straight back. StaticPopup "DELETE_ITEM" calls this on Accept, so the whole
// drag-an-item-out-to-destroy-it gesture ended in the item reappearing.
//
// cursorWireSlot already converts the held position into the bag and slot the
// server expects, which is the same translation the drop half of a drag uses.
static int lua_DeleteCursorItem(lua_State* L) {
    auto* gh = getGameHandler(L);
    uint8_t bag = 0, slot = 0;
    if (gh && cursorWireSlot(bag, slot)) {
        // The whole stack: WoW asks about a partial stack before it gets here,
        // and the popup that calls this is the confirmation for all of it.
        gh->destroyItem(bag, slot, 0);
    }
    clearCursorItem(L);
    return 0;
}

// AutoEquipCursorItem() - equip item from cursor
static int lua_AutoEquipCursorItem(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (gh && s_cursorType == CursorType::ITEM && s_cursorId != 0) {
        gh->useItemById(s_cursorId);
    }
    clearCursorItem(L);
    return 0;
}

// --- Frame System ---
// Minimal WoW-compatible frame objects with RegisterEvent/SetScript/GetScript.
// Frames are Lua tables with a metatable that provides methods.

// Frame method: frame:RegisterEvent("EVENT")

// The modifier state comes from the keyboard rather than from ImGui, which
// learns it from events this client does not always route there once FrameXML
// owns the mouse. lua_system_api registers the same three names against
// SDL_GetModState; these agree with it now instead of racing it, and the
// duplicate Is*KeyDown bindings that used to live here are gone with them.
static bool shiftHeld() { return (SDL_GetModState() & KMOD_SHIFT) != 0; }
static bool ctrlHeld()  { return (SDL_GetModState() & KMOD_CTRL)  != 0; }
static bool altHeld()   { return (SDL_GetModState() & KMOD_ALT)   != 0; }


/// What bindings.xml declares for a modified-click action, or empty.
///
/// One list, read once. The emitter keeps every <ModifiedClick action= default=>
/// in __WoweeModifiedClick, which is the same file the key bindings panel is
/// built from - so this cannot drift from what the interface believes.
///
/// It had drifted. A table written out by hand here answered ALT for FOCUSCAST,
/// which bindings.xml declares as NONE, so alt-clicking a spell tried to cast
/// it at the focus when nothing had asked for that; and both flyout actions are
/// declared ALT and fell through to a shift default.
static std::string modifiedClickBinding(lua_State* L, const std::string& action) {
    lua_getglobal(L, "__WoweeModifiedClick");
    if (!lua_istable(L, -1)) { lua_pop(L, 1); return {}; }
    lua_getfield(L, -1, action.c_str());
    const char* value = lua_tostring(L, -1);
    std::string out = value ? value : "";
    lua_pop(L, 2);
    return out;
}

/// Whether the modifiers a binding names are all held.
///
/// A binding is a dash-separated list ending in an optional button -
/// "SHIFT-BUTTON1", "CTRL", "NONE". The button half is the caller's business:
/// it already knows which button was pressed, and every caller here is inside
/// a handler for one.
static bool modifiersHeldFor(const std::string& binding) {
    if (binding.empty() || binding == "NONE") return false;
    bool wantShift = false, wantCtrl = false, wantAlt = false;
    std::string wantButton;
    size_t at = 0;
    while (at <= binding.size()) {
        const size_t dash = binding.find('-', at);
        const std::string part = binding.substr(
            at, dash == std::string::npos ? std::string::npos : dash - at);
        if (part == "SHIFT") wantShift = true;
        else if (part == "CTRL") wantCtrl = true;
        else if (part == "ALT") wantAlt = true;
        // A binding names a button as often as it names a modifier, and the
        // two named actions on the same modifier are told apart by nothing
        // else: CHATLINK is SHIFT-BUTTON1 and SOCKETITEM is SHIFT-BUTTON2.
        // Reading only the modifier made both true on any shift-click, so
        // shift-clicking an item in a bag put the link in chat and opened the
        // socketing window behind it - or, when the link could not be
        // inserted, only the socketing window.
        else if (part == "BUTTON1") wantButton = "LeftButton";
        else if (part == "BUTTON2") wantButton = "RightButton";
        if (dash == std::string::npos) break;
        at = dash + 1;
    }
    if (!wantShift && !wantCtrl && !wantAlt) return false;
    if (!wantButton.empty()) {
        // Only while a click is being dispatched. Asked at any other moment -
        // an OnUpdate deciding what a tooltip should say - the modifier alone
        // is the whole of the question, and the button has not been pressed.
        const std::string& clicking = currentClickButton();
        if (!clicking.empty() && clicking != wantButton) return false;
    }
    return (!wantShift || shiftHeld()) && (!wantCtrl || ctrlHeld()) &&
           (!wantAlt || altHeld());
}

// IsModifiedClick(action) → boolean
static int lua_IsModifiedClick(lua_State* L) {
    const char* action = luaL_optstring(L, 1, "");
    std::string act(action);
    for (char& c : act) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    if (act.empty()) {
        // No action named means "is this click modified at all", which is how
        // the paperdoll and the bags branch: IsModifiedClick() picks between
        // OnModifiedClick and the plain OnClick. Answering with shift alone
        // sent every ctrl-click down the plain path, so ctrl-clicking a worn
        // item picked it up instead of putting it in the dressing room.
        lua_pushboolean(L, (shiftHeld() || ctrlHeld() || altHeld()) ? 1 : 0);
        return 1;
    }
    const std::string binding = modifiedClickBinding(L, act);
    // An action bindings.xml has never heard of. Shift is what the interface's
    // own unbound actions use, and answering false would make a modified click
    // on one unreachable rather than merely bound elsewhere.
    lua_pushboolean(L, (binding.empty() ? shiftHeld() : modifiersHeldFor(binding)) ? 1 : 0);
    return 1;
}

// GetModifiedClick(action) → the binding it is on ("SHIFT", "CTRL-BUTTON1", "NONE")
static int lua_GetModifiedClick(lua_State* L) {
    const char* action = luaL_optstring(L, 1, "");
    std::string act(action);
    for (char& c : act) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    const std::string binding = modifiedClickBinding(L, act);
    lua_pushstring(L, binding.empty() ? "SHIFT" : binding.c_str());
    return 1;
}

// SetModifiedClick(action, binding) - rebinding one from the options panel.
//
// It wrote nowhere, so the panel accepted a change and the next question about
// that action answered the old value. The table the emitter fills is the same
// one read above, so writing to it is the whole of it.
static int lua_SetModifiedClick(lua_State* L) {
    const char* action = luaL_optstring(L, 1, "");
    const char* binding = luaL_optstring(L, 2, "NONE");
    if (!action || !*action) return 0;
    std::string act(action);
    for (char& c : act) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    lua_getglobal(L, "__WoweeModifiedClick");
    if (!lua_istable(L, -1)) { lua_pop(L, 1); return 0; }
    lua_pushstring(L, binding);
    lua_setfield(L, -2, act.c_str());
    lua_pop(L, 1);
    return 0;
}


// --- Trading ---
//
// Placing an item is the only part that needs the cursor, which is why these
// live here. The peer's side is read-only: a trade slot of theirs can be looked
// at and not touched, so clicking one does nothing rather than pretending.

static int pushTradeSlot(lua_State* L, game::GameHandler* gh,
                         const game::GameHandler::TradeSlot& slot) {
    if (!slot.occupied || slot.itemId == 0) { return luaReturnNil(L); }
    const auto* info = gh->getItemInfo(slot.itemId);
    lua_pushstring(L, info ? info->name.c_str() : "");
    lua_pushstring(L, gh->getItemIconPath(
        info && info->displayInfoId ? info->displayInfoId : slot.displayId).c_str());
    lua_pushnumber(L, slot.stackCount);
    lua_pushboolean(L, 1);       // isUsable
    lua_pushnil(L);              // enchantment: not carried by the trade packet
    return 5;
}

static int lua_GetTradePlayerItemInfo(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int i = static_cast<int>(luaL_optnumber(L, 1, 0));
    if (!gh || i < 1 || i > game::GameHandler::TRADE_SLOT_COUNT) { return luaReturnNil(L); }
    return pushTradeSlot(L, gh, gh->getMyTradeSlots()[i - 1]);
}

static int lua_GetTradeTargetItemInfo(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int i = static_cast<int>(luaL_optnumber(L, 1, 0));
    if (!gh || i < 1 || i > game::GameHandler::TRADE_SLOT_COUNT) { return luaReturnNil(L); }
    return pushTradeSlot(L, gh, gh->getPeerTradeSlots()[i - 1]);
}

static int pushTradeLink(lua_State* L, game::GameHandler* gh,
                         const game::GameHandler::TradeSlot& slot) {
    const auto* info = (gh && slot.occupied) ? gh->getItemInfo(slot.itemId) : nullptr;
    if (!info || info->name.empty()) { return luaReturnNil(L); }
    const std::string link = game::itemChatLink(slot.itemId, static_cast<uint32_t>(info->quality), info->name);
    lua_pushstring(L, link.c_str());
    return 1;
}

static int lua_GetTradePlayerItemLink(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int i = static_cast<int>(luaL_optnumber(L, 1, 0));
    if (!gh || i < 1 || i > game::GameHandler::TRADE_SLOT_COUNT) { return luaReturnNil(L); }
    return pushTradeLink(L, gh, gh->getMyTradeSlots()[i - 1]);
}

static int lua_GetTradeTargetItemLink(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int i = static_cast<int>(luaL_optnumber(L, 1, 0));
    if (!gh || i < 1 || i > game::GameHandler::TRADE_SLOT_COUNT) { return luaReturnNil(L); }
    return pushTradeLink(L, gh, gh->getPeerTradeSlots()[i - 1]);
}

// ClickTradeButton(slot) - put what is held into the slot, or take back what is
// already there when nothing is held
static int lua_ClickTradeButton(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int i = static_cast<int>(luaL_optnumber(L, 1, 0));
    if (!gh || i < 1 || i > game::GameHandler::TRADE_SLOT_COUNT) return 0;
    uint8_t srcBag = 0, srcSlot = 0;
    // Through the same translation as every other send. This passed FrameXML's
    // own numbering straight to the server - bags counted 0 to 4 and slots from
    // one - where the wire wants the flat space GetItemByPos reads: container
    // 255 with an absolute slot, or a worn bag's equipment slot with a 0-based
    // index inside it. The server finds no item at the slot it was given and
    // answers TRADE_STATUS_TRADE_CANCELED, so offering an item ended the trade.
    if (s_cursorType == CursorType::ITEM && s_cursorBag >= 0 &&
        cursorWireSlot(srcBag, srcSlot)) {
        gh->setTradeItem(static_cast<uint8_t>(i - 1), srcBag, srcSlot);
        clearCursorItem(L);
    } else {
        gh->clearTradeItem(static_cast<uint8_t>(i - 1));
    }
    return 0;
}

// The other side's slots belong to the other player.
static int lua_ClickTargetTradeButton(lua_State* L) { (void)L; return 0; }

static int lua_SetTradeMoney(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (gh) gh->setTradeGold(static_cast<uint64_t>(luaL_optnumber(L, 1, 0)));
    return 0;
}

static int lua_CloseTrade(lua_State* L) {
    if (auto* gh = getGameHandler(L)) gh->cancelTrade();
    return 0;
}

static int lua_BeginTrade(lua_State* L) {
    if (auto* gh = getGameHandler(L)) gh->acceptTradeRequest();
    return 0;
}

// --- Keybinding API ---
//
// Which commands exist, and in what order, comes from bindings.xml: the emitter
// turns it into __WoweeBindings (the list the UI walks, header rows included)
// and __WoweeBindingScripts (what each one runs). What each is *bound to* lives
// here instead, because it outlives any one Lua state and has to reach a file.

namespace {

/// command → its two keys, empty meaning not bound. Two because the UI offers a
/// primary and a secondary and will write either.
std::map<std::string, std::array<std::string, 2>>& bindingKeys() {
    static std::map<std::string, std::array<std::string, 2>> keys;
    return keys;
}

std::string bindingsFilePath() {
    return core::getConfigRoot() + "/bindings.cfg";
}

/// ImGui spells keys as "C" and "Space"; the interface expects "C" and "SPACE",
/// and compares them as strings everywhere.
std::string wowKeyFromImGui(ImGuiKey key) {
    // An action with nothing bound to it answers "None", which uppercased is a
    // perfectly ordinary-looking key name and would show as one.
    if (key == ImGuiKey_None) return "";
    const char* name = ImGui::GetKeyName(key);
    if (!name || !*name) return "";
    std::string out(name);
    for (char& c : out) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return out;
}

/// The commands the client has a real action behind. Rebinding one of these has
/// to reach the manager, or the list would show the new key while the client
/// went on answering to the old one.
struct LiveBinding {
    const char* command;
    wowee::ui::KeybindingManager::Action action;
};
const LiveBinding kLiveBindings[] = {
    {"TOGGLECHARACTER0",  wowee::ui::KeybindingManager::Action::TOGGLE_CHARACTER_SCREEN},
    {"TOGGLEBACKPACK",    wowee::ui::KeybindingManager::Action::TOGGLE_BAGS},
    {"TOGGLESPELLBOOK",   wowee::ui::KeybindingManager::Action::TOGGLE_SPELLBOOK},
    {"TOGGLETALENTS",     wowee::ui::KeybindingManager::Action::TOGGLE_TALENTS},
    {"TOGGLEQUESTLOG",    wowee::ui::KeybindingManager::Action::TOGGLE_QUESTS},
    {"TOGGLEWORLDMAP",    wowee::ui::KeybindingManager::Action::TOGGLE_WORLD_MAP},
    {"TOGGLEMINIMAP",     wowee::ui::KeybindingManager::Action::TOGGLE_MINIMAP},
    {"TOGGLEACHIEVEMENT", wowee::ui::KeybindingManager::Action::TOGGLE_ACHIEVEMENTS},
    {"TOGGLEGUILDTAB",    wowee::ui::KeybindingManager::Action::TOGGLE_GUILD_ROSTER},
};

/// The named key whose name matches, or none. ImGui offers no reverse lookup,
/// and the set is small enough that walking it costs nothing next to the file
/// write that follows a rebind.
ImGuiKey imGuiKeyFromWow(const std::string& name) {
    if (name.empty()) return ImGuiKey_None;
    for (int k = ImGuiKey_NamedKey_BEGIN; k < ImGuiKey_NamedKey_END; ++k) {
        if (wowKeyFromImGui(static_cast<ImGuiKey>(k)) == name) {
            return static_cast<ImGuiKey>(k);
        }
    }
    return ImGuiKey_None;
}

/// Tells the client what a command is bound to now, for the commands it acts
/// on. Silent for the rest, which are listed and saved but not yet answered.
/// Hand a binding to the client's own keybinding manager, if it answers that
/// command at all.
///
/// Two things do not survive this, both worth knowing before chasing either as
/// a bug.
///
/// A second key for one command is accepted, works for the session and is gone
/// at the next start. The interface allows two per command and this manager
/// holds one per action, so what is pushed here is keys[0] - the primary - and
/// the other lives only in the Lua map above, which nothing saves. Setting a
/// primary does persist: rebinding the bags from B to N writes toggle_bags=N
/// and comes back as N. Making the second key stick means the manager holding
/// two, which is a change to what a binding is here rather than a repair.
///
/// A binding for a command not in kLiveBindings does not reach the client at
/// all, and is not saved either. That is the intended half of this: those are
/// the commands the interface answers for itself.
void pushBindingToClient(const std::string& command, const std::string& key) {
    for (const auto& live : kLiveBindings) {
        if (command != live.command) continue;
        const ImGuiKey imKey = imGuiKeyFromWow(key);
        wowee::ui::KeybindingManager::getInstance().setKeyForAction(live.action, imKey);
        return;
    }
}

/// The key the client is listening for, when this is a command it acts on.
///
/// Read at the moment it is asked rather than from the copy below, because the
/// client's own settings panel can rebind these too - and then the copy is a
/// second answer to a question the manager already owns, one keystroke out of
/// date and shown on every action button.
std::optional<std::string> liveKeyFor(const std::string& command) {
    for (const auto& live : kLiveBindings) {
        if (command != live.command) continue;
        std::string key =
            wowKeyFromImGui(wowee::ui::KeybindingManager::getInstance()
                                .getKeyForAction(live.action));
        if (key.empty()) return std::nullopt;
        return key;
    }
    return std::nullopt;
}

/// The keys the client is actually listening for, asked of the manager that
/// listens rather than restated here - a second copy would be wrong the moment
/// either side moved. Commands the client has no action for keep their retail
/// default, so the list reads correctly even where nothing acts on it yet.
void seedBindingDefaults() {
    auto& keys = bindingKeys();
    if (!keys.empty()) return;

    static const struct { const char* command; const char* key; } kDefaults[] = {
        {"MOVEFORWARD", "W"},   {"MOVEBACKWARD", "S"},
        {"TURNLEFT", "A"},      {"TURNRIGHT", "D"},
        {"STRAFELEFT", "Q"},    {"STRAFERIGHT", "E"},
        {"JUMP", "SPACE"},      {"TOGGLEAUTORUN", "NUMLOCK"},
        {"TOGGLEGAMEMENU", "ESCAPE"},
        {"SCREENSHOT", "PRINTSCREEN"},
        {"ACTIONBUTTON1", "1"}, {"ACTIONBUTTON2", "2"},
        {"ACTIONBUTTON3", "3"}, {"ACTIONBUTTON4", "4"},
        {"ACTIONBUTTON5", "5"}, {"ACTIONBUTTON6", "6"},
        {"ACTIONBUTTON7", "7"}, {"ACTIONBUTTON8", "8"},
        {"ACTIONBUTTON9", "9"}, {"ACTIONBUTTON10", "0"},
        {"ACTIONBUTTON11", "-"},{"ACTIONBUTTON12", "="},
    };
    for (const auto& d : kDefaults) keys[d.command] = {d.key, ""};

    // Where a command corresponds to something the client really does, the key
    // shown is the one it really answers to.
    auto& manager = wowee::ui::KeybindingManager::getInstance();
    for (const auto& live : kLiveBindings) {
        const std::string key = wowKeyFromImGui(manager.getKeyForAction(live.action));
        if (!key.empty()) keys[live.command] = {key, ""};
    }
}

/// The command list the emitter built, or zero if bindings.xml never loaded.
int bindingCount(lua_State* L) {
    lua_getglobal(L, "__WoweeBindings");
    if (!lua_istable(L, -1)) { lua_pop(L, 1); return 0; }
    const int n = static_cast<int>(lua_objlen(L, -1));
    lua_pop(L, 1);
    return n;
}

/// The command at a one-based position, empty when out of range.
std::string bindingAt(lua_State* L, int index) {
    lua_getglobal(L, "__WoweeBindings");
    if (!lua_istable(L, -1)) { lua_pop(L, 1); return ""; }
    lua_rawgeti(L, -1, index);
    const char* s = lua_tostring(L, -1);
    std::string out = s ? s : "";
    lua_pop(L, 2);
    return out;
}

}  // namespace

bool clientActsOnBinding(const std::string& command) {
    // The nine the client really performs, asked of the same list that keeps
    // their keys in step rather than restated - a second copy would be wrong
    // the moment either side moved.
    for (const auto& live : kLiveBindings) {
        if (command == live.command) return true;
    }
    // And the rest of what the client answers without going through a binding
    // at all. Each is a real call site: movement and jumping are polled every
    // frame, the action bar takes 1 through 0 and the two beside them in
    // GameScreen, chat opens on Enter and on slash, Tab targets, and Escape has
    // a whole chain of its own in escape_action.hpp.
    //
    // By command rather than by key, because the conflict is that the *action*
    // happens twice. Most bindings toggle something, so two answers to one
    // press cancel out and the key reads as dead - which is worse than doing
    // nothing, since it looks like the binding never ran.
    static constexpr const char* kClientAnswers[] = {
        "MOVEFORWARD", "MOVEBACKWARD", "TURNLEFT", "TURNRIGHT",
        "STRAFELEFT", "STRAFERIGHT", "JUMP", "TOGGLEAUTORUN", "TOGGLERUN",
        "TOGGLEGAMEMENU", "OPENCHAT", "OPENCHATSLASH", "TARGETNEARESTENEMY",
        "SCREENSHOT", "TOGGLESHEATH",
        "ACTIONBUTTON1", "ACTIONBUTTON2", "ACTIONBUTTON3", "ACTIONBUTTON4",
        "ACTIONBUTTON5", "ACTIONBUTTON6", "ACTIONBUTTON7", "ACTIONBUTTON8",
        "ACTIONBUTTON9", "ACTIONBUTTON10", "ACTIONBUTTON11", "ACTIONBUTTON12",
    };
    for (const char* name : kClientAnswers) {
        if (command == name) return true;
    }
    return false;
}

// GetBindingKey(command) → key1, key2 (or nil)
static int lua_GetBindingKey(lua_State* L) {
    seedBindingDefaults();
    const std::string command = luaL_checkstring(L, 1);
    if (auto live = liveKeyFor(command)) {
        lua_pushstring(L, live->c_str());
        lua_pushnil(L);
        return 2;
    }
    auto it = bindingKeys().find(command);
    if (it == bindingKeys().end()) { lua_pushnil(L); lua_pushnil(L); return 2; }
    for (const std::string& key : it->second) {
        if (key.empty()) lua_pushnil(L); else lua_pushstring(L, key.c_str());
    }
    return 2;
}

// GetBindingAction(key) → command (or nil)
static int lua_GetBindingAction(lua_State* L) {
    seedBindingDefaults();
    const std::string key = luaL_checkstring(L, 1);
    for (const auto& [command, keys] : bindingKeys()) {
        if (keys[0] == key || keys[1] == key) {
            lua_pushstring(L, command.c_str());
            return 1;
        }
    }
    lua_pushnil(L);
    return 1;
}

// GetNumBindings() → how many rows the list has, headers included
static int lua_GetNumBindings(lua_State* L) {
    lua_pushinteger(L, bindingCount(L));
    return 1;
}

// GetBinding(index) → command, key1, key2. A header row is a command like
// "HEADER_MOVEMENT" with no keys, which is how the UI tells the two apart.
static int lua_GetBinding(lua_State* L) {
    seedBindingDefaults();
    const int index = static_cast<int>(luaL_checkinteger(L, 1));
    const std::string command = bindingAt(L, index);
    if (command.empty()) { lua_pushnil(L); return 1; }
    lua_pushstring(L, command.c_str());
    if (auto live = liveKeyFor(command)) {
        lua_pushstring(L, live->c_str());
        lua_pushnil(L);
        return 3;
    }
    auto it = bindingKeys().find(command);
    if (it == bindingKeys().end()) { lua_pushnil(L); lua_pushnil(L); return 3; }
    for (const std::string& key : it->second) {
        if (key.empty()) lua_pushnil(L); else lua_pushstring(L, key.c_str());
    }
    return 3;
}

// SetBinding(key, command) → true. A nil command clears whatever holds the key,
// which is how the UI unbinds.
static int lua_SetBinding(lua_State* L) {
    seedBindingDefaults();
    const std::string key = luaL_checkstring(L, 1);
    const char* command = lua_isnoneornil(L, 2) ? nullptr : luaL_checkstring(L, 2);

    // A key belongs to one command at a time, so it leaves the one that had it
    // before it joins another - otherwise both claim it and which one answers
    // depends on map order.
    for (auto& [existing, keys] : bindingKeys()) {
        for (std::string& slot : keys) {
            if (slot != key) continue;
            slot.clear();
            // The command that just lost the key has to be told, not only the
            // one that gained it, or the client answers to both. What it
            // answers to now is whichever slot still holds something, which is
            // not always the first.
            pushBindingToClient(existing, keys[0].empty() ? keys[1] : keys[0]);
        }
    }
    if (command) {
        auto& keys = bindingKeys()[command];
        if (keys[0].empty()) keys[0] = key; else keys[1] = key;
        pushBindingToClient(command, keys[0]);
    }
    // Six frames wait on this, the action buttons among them: it is what
    // redraws the little key printed in the corner. Without it a rebind takes
    // effect while every button goes on showing the old key.
    if (auto* gh = getGameHandler(L)) gh->fireAddonEvent("UPDATE_BINDINGS", {});
    lua_pushboolean(L, 1);
    return 1;
}

// SaveBindings(which) → writes what is bound where, for the next run
static int lua_SaveBindings(lua_State* L) {
    (void)L;
    const std::string path = bindingsFilePath();
    std::error_code ec;
    std::filesystem::create_directories(
        std::filesystem::path(path).parent_path(), ec);
    std::ofstream out(path, std::ios::trunc);
    if (!out) {
        LOG_WARNING("Could not write the key bindings to ", path);
        return 0;
    }
    for (const auto& [command, keys] : bindingKeys()) {
        // Both slots on one line, the second empty when there is no second key,
        // so a command that lost one is recorded as having lost it.
        out << command << "=" << keys[0] << "," << keys[1] << "\n";
    }
    return 0;
}

// LoadBindings(which) → what a previous run saved, over the defaults
static int lua_LoadBindings(lua_State* L) {
    (void)L;
    seedBindingDefaults();
    std::ifstream in(bindingsFilePath());
    if (!in) return 0;   // Nothing saved yet is not a failure.
    std::string line;
    while (std::getline(in, line)) {
        const size_t eq = line.find('=');
        if (eq == std::string::npos) continue;
        const std::string command = line.substr(0, eq);
        const std::string rest = line.substr(eq + 1);
        const size_t comma = rest.find(',');
        bindingKeys()[command] = {
            rest.substr(0, comma),
            comma == std::string::npos ? "" : rest.substr(comma + 1)
        };
        // What was saved is what the client should answer to, not what it
        // started with - otherwise a rebind survives in the list and nowhere
        // else, and only until the next save overwrites it.
        pushBindingToClient(command, bindingKeys()[command][0]);
    }
    if (auto* gh = getGameHandler(L)) gh->fireAddonEvent("UPDATE_BINDINGS", {});
    return 0;
}

// RunBinding(command, keystate) → runs what bindings.xml says the command does
static int lua_RunBinding(lua_State* L) {
    const std::string command = luaL_checkstring(L, 1);
    const char* keystate = lua_isnoneornil(L, 2) ? "down" : luaL_checkstring(L, 2);

    lua_getglobal(L, "__WoweeBindingScripts");
    if (!lua_istable(L, -1)) { lua_pop(L, 1); return 0; }
    lua_getfield(L, -1, command.c_str());
    if (!lua_isfunction(L, -1)) { lua_pop(L, 2); return 0; }
    lua_pushstring(L, keystate);
    if (lua_pcall(L, 1, 0, 0) != 0) {
        LOG_WARNING("Binding ", command, " failed: ",
                    lua_tostring(L, -1) ? lua_tostring(L, -1) : "?");
        lua_pop(L, 1);
    }
    lua_pop(L, 1);
    return 0;
}
// The override-binding family, none of which this client implements - only
// the Click variant was ever *bound*, and the other four raised. They are
// reached from restrictedframes.lua, the secure-frame machinery behind action
// buttons and unit frames, so a raise there breaks the frame rather than the
// binding. Bound alongside it, doing the same nothing.
static int lua_SetOverrideBindingClick(lua_State* L) { (void)L; return 0; }
static int lua_SetOverrideBinding(lua_State* L) { (void)L; return 0; }
static int lua_SetOverrideBindingSpell(lua_State* L) { (void)L; return 0; }
static int lua_SetOverrideBindingMacro(lua_State* L) { (void)L; return 0; }
static int lua_SetOverrideBindingItem(lua_State* L) { (void)L; return 0; }
static int lua_ClearOverrideBindings(lua_State* L) { (void)L; return 0; }

// Frame methods: SetPoint, SetSize, SetWidth, SetHeight, GetWidth, GetHeight, GetCenter, SetAlpha, GetAlpha

void registerActionLuaAPI(lua_State* L) {
    static const struct { const char* name; lua_CFunction func; } api[] = {
                {"HasAction",           lua_HasAction},
                // A macro's name, which only a macro has; ActionButton_Update
                // shows it under the icon and expects nothing for a spell.
                // GetActionText(slot) → the name written under the button.
                //
                // Only a macro has one, which is what the action button draws
                // in its Name font string - so with nil answered for every slot
                // a bar full of macros was a row of nameless icons, all of them
                // the same question mark unless the player had picked an icon.
                // The names were there: GetMacroInfo has been reading them off
                // the same handler all along.
                {"GetActionText", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int slot = static_cast<int>(luaL_optnumber(L, 1, 0)) - 1;
            if (!gh || slot < 0) { lua_pushnil(L); return 1; }
            const auto& bar = gh->getActionBar();
            if (slot >= static_cast<int>(bar.size())) { lua_pushnil(L); return 1; }
            const auto& action = bar[static_cast<size_t>(slot)];
            if (action.type != game::ActionBarSlot::MACRO) { lua_pushnil(L); return 1; }
            const std::string name = gh->getMacroName(action.id);
            if (name.empty()) { lua_pushnil(L); return 1; }
            lua_pushstring(L, name.c_str());
            return 1;
        }},
                // Whether the action has a range to be in or out of. Nothing
                // here tracks that yet, and false is what "no range check"
                // looks like - the button simply never dims for distance.
                {"ActionHasRange",      [](lua_State* L) -> int {
                    lua_pushboolean(L, 0);
                    return 1;
                }},
                {"GetActionTexture",    lua_GetActionTexture},
                {"IsCurrentAction",     lua_IsCurrentAction},
                {"IsUsableAction",      lua_IsUsableAction},
                {"IsActionInRange",     lua_IsActionInRange},
                {"GetActionInfo",       lua_GetActionInfo},
                {"GetActionCount",      lua_GetActionCount},
                {"GetActionCooldown",   lua_GetActionCooldown},
                {"UseAction",           lua_UseAction},
                {"PickupAction",        lua_PickupAction},
                {"PlaceAction",         lua_PlaceAction},
                {"PickupSpell",         lua_PickupSpell},
                {"PickupCompanion",     lua_PickupCompanion},
                {"PickupGuildBankItem", lua_PickupGuildBankItem},
                {"PlaceGlyphInSocket",  lua_PlaceGlyphInSocket},
                {"SplitGuildBankItem",  lua_SplitGuildBankItem},
                {"PickupSpellBookItem", lua_PickupSpellBookItem},
                {"PickupContainerItem", lua_PickupContainerItem},
                {"PickupItem",          lua_PickupItem},
                // The totem bar's summon flyout calls these two from its
                // OnClick and OnLeave, and nothing in this interface defines
                // them - not multicastactionbarframe.lua beside the XML that
                // calls them, not anywhere. An upstream gap rather than one of
                // this client's, but the raise lands here all the same, and it
                // lands on a shaman clicking their own totem bar.
                //
                // Answered with nothing, which makes the button inert. There
                // is no honest alternative: the missing function is what would
                // have decided which totem the flyout chose, and inventing one
                // would summon whatever this guessed.
                {"MultiCastSummonSpellButtonFlyoutButton_OnClick", lua_ReturnNothing2},
                {"MultiCastSummonSpellButtonFlyoutButton_OnLeave", lua_ReturnNothing2},
                {"PickupInventoryItem", lua_PickupInventoryItem},
                {"PickupBagFromSlot",   lua_PickupBagFromSlot},
                {"ClickSendMailItemButton", lua_ClickSendMailItemButton},
                {"ClearCursor",         lua_ClearCursor},
                {"GetCursorInfo",       lua_GetCursorInfo},
                {"PickupPetAction",     lua_PickupPetAction},
                {"CursorHasItem",       lua_CursorHasItem},
                {"CursorHasSpell",      lua_CursorHasSpell},
                {"DeleteCursorItem",    lua_DeleteCursorItem},
                {"AutoEquipCursorItem", lua_AutoEquipCursorItem},
                {"IsModifiedClick",     lua_IsModifiedClick},
                {"GetModifiedClick",    lua_GetModifiedClick},
                {"SetModifiedClick",    lua_SetModifiedClick},
                // Macros. The client has stored their text all along and
                // persisted it per character; only the interface's way in was
                // missing, so GetNumMacros answered zero and the panel opened
                // empty over a full set.
                //
                // WoW splits the ids: 1-36 are account-wide, 37 and up belong
                // to the character. That split is what GetNumMacros reports and
                // what CreateMacro's perCharacter flag chooses between.
                {"GetNumMacros",        [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            int global = 0, perChar = 0;
            if (gh) {
                for (uint32_t id : gh->getMacroIds()) {
                    if (id <= 36) ++global; else ++perChar;
                }
            }
            lua_pushnumber(L, global);
            lua_pushnumber(L, perChar);
            return 2;
        }},
                // GetMacroInfo(id) → name, icon, body
                {"GetMacroInfo",        [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const uint32_t id = static_cast<uint32_t>(luaL_optnumber(L, 1, 0));
            if (!gh || id == 0 || gh->getMacroText(id).empty()) {
                return luaReturnNil(L);
            }
            std::string name = gh->getMacroName(id);
            if (name.empty()) name = "Macro";
            std::string icon = gh->getMacroIcon(id);
            if (icon.empty()) icon = "Interface\\Icons\\INV_Misc_QuestionMark";
            lua_pushstring(L, name.c_str());
            lua_pushstring(L, icon.c_str());
            lua_pushstring(L, gh->getMacroText(id).c_str());
            return 3;
        }},
                // CreateMacro(name, icon, body, perCharacter) → id
                {"CreateMacro",         [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) return luaReturnNil(L);
            const char* name = luaL_optstring(L, 1, "Macro");
            const char* icon = luaL_optstring(L, 2, "");
            const char* body = luaL_optstring(L, 3, "");
            const bool perChar = lua_toboolean(L, 4) != 0;
            // The first free id in the half that was asked for. WoW allows 18
            // of each; refusing past that is what stops a full list growing
            // ids the interface will not show.
            const uint32_t first = perChar ? 37u : 1u;
            const uint32_t last  = perChar ? 54u : 36u;
            uint32_t id = 0;
            for (uint32_t candidate = first; candidate <= last; ++candidate) {
                if (gh->getMacroText(candidate).empty()) { id = candidate; break; }
            }
            if (id == 0) return luaReturnNil(L);
            gh->setMacroText(id, body);
            gh->setMacroMeta(id, name, icon);
            lua_pushnumber(L, id);
            return 1;
        }},
                // EditMacro(id, name, icon, body) → id
                {"EditMacro",           [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const uint32_t id = static_cast<uint32_t>(luaL_optnumber(L, 1, 0));
            if (!gh || id == 0) return luaReturnNil(L);
            const char* name = luaL_optstring(L, 2, nullptr);
            const char* icon = luaL_optstring(L, 3, nullptr);
            const char* body = luaL_optstring(L, 4, nullptr);
            // Each part is optional and nil means "leave it": EditMacro is
            // called with only a name when a macro is renamed, and writing the
            // other two as empty would erase the macro being renamed.
            if (body) gh->setMacroText(id, body);
            gh->setMacroMeta(id, name ? name : gh->getMacroName(id),
                                 icon ? icon : gh->getMacroIcon(id));
            lua_pushnumber(L, id);
            return 1;
        }},
                // DeleteMacro(index) - and the rest of the half shifts down.
                //
                // Every macro verb here takes what WoW calls an index and what
                // this client stores as an id, and the two are the same number
                // only while the slots are dense. CreateMacro fills the first
                // free slot, so they start dense; this emptied one and left a
                // hole, and from then on the two disagreed.
                //
                // What that costs: with macros in slots 1, 2 and 3, deleting
                // the second leaves 1 and 3. GetNumMacros counts two, so the
                // macro frame asks for slots 1 and 2 - one macro, one blank
                // row, and the macro in slot 3 invisible and unreachable.
                //
                // Compacting is also what WoW does. Deleting a macro there
                // shifts the ones after it down, which is why a macro on an
                // action bar can move when an earlier one is deleted; matching
                // that is more faithful than keeping a hole, and it is the
                // assumption getMacroIds already documents.
                {"DeleteMacro",         [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const uint32_t id = static_cast<uint32_t>(luaL_optnumber(L, 1, 0));
            if (!gh || id == 0) return 0;
            // Which half this slot belongs to; the two do not shift into
            // each other.
            const uint32_t first = (id >= 37u) ? 37u : 1u;
            const uint32_t last  = (id >= 37u) ? 54u : 36u;
            if (id < first || id > last) return 0;
            for (uint32_t slot = id; slot < last; ++slot) {
                const std::string next = gh->getMacroText(slot + 1);
                if (next.empty()) {
                    gh->setMacroText(slot, "");
                    gh->setMacroMeta(slot, "", "");
                    break;
                }
                gh->setMacroText(slot, next);
                gh->setMacroMeta(slot, gh->getMacroName(slot + 1),
                                       gh->getMacroIcon(slot + 1));
            }
            // The last slot in the half is never written by the loop above.
            gh->setMacroText(last, "");
            gh->setMacroMeta(last, "", "");
            return 0;
        }},
                // Saved as each change is made, so this has nothing to do.
                {"SaveMacros",          [](lua_State* L) -> int { (void)L; return 0; }},
                // PickupMacro(id) - onto the cursor, so it can be dropped on
                // an action slot. The slot type already existed and the packet
                // that sets it already went out for spells and items; a macro
                // was simply never something the cursor could hold.
                {"PickupMacro",         [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const uint32_t id = static_cast<uint32_t>(luaL_optnumber(L, 1, 0));
            if (!gh || id == 0 || gh->getMacroText(id).empty()) return 0;
            setCursorType(L, CursorType::MACRO);
            s_cursorId = id;
            std::string icon = gh->getMacroIcon(id);
            if (icon.empty()) icon = "Interface\\Icons\\INV_Misc_QuestionMark";
            wowee::ui::frameXmlSetCursorItem(icon);
            return 0;
        }},
                {"GetTradePlayerItemInfo", lua_GetTradePlayerItemInfo},
                {"GetTradeTargetItemInfo", lua_GetTradeTargetItemInfo},
                {"GetTradePlayerItemLink", lua_GetTradePlayerItemLink},
                {"GetTradeTargetItemLink", lua_GetTradeTargetItemLink},
                {"ClickTradeButton",    lua_ClickTradeButton},
                {"ClickTargetTradeButton", lua_ClickTargetTradeButton},
                {"SetTradeMoney",       lua_SetTradeMoney},
                // CancelTradeAccept() - take an acceptance back without
                // closing the trade. The cancel button does this instead of
                // closing while the player has already accepted, so without it
                // the only way out of an accepted trade was to end it.
                {"CancelTradeAccept", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L)) gh->unacceptTrade();
            return 0;
        }},
                {"CloseTrade",          lua_CloseTrade},
                {"BeginTrade",          lua_BeginTrade},
                {"GetBindingKey",       lua_GetBindingKey},
                {"GetBindingAction",    lua_GetBindingAction},
                {"GetNumBindings",      lua_GetNumBindings},
                {"GetBinding",          lua_GetBinding},
                {"SetBinding",          lua_SetBinding},
                // GetBindingByKey(key) - the binding a key runs, or nil.
                //
                // Reached from GetBindingFromClick, which StaticPopup_OnKeyDown
                // calls on *every* keypress while a popup is up - so with
                // dialogs handed over, pressing any key with one on screen hit
                // an unbound global and raised. Escape is how a popup is
                // dismissed, so it was the keypress most likely to hit it.
                //
                // Two names are all FrameXML compares against here, and both
                // are fixed in this client: escape opens the game menu and
                // print screen takes a screenshot, neither rebindable. Anything
                // else answers nil, which is what WoW answers for an unbound
                // key and which lets the dialog's own key handling run.
                {"GetBindingByKey", [](lua_State* L) -> int {
            std::string key(luaL_optstring(L, 1, ""));
            for (char& c : key) c = static_cast<char>(std::toupper(
                static_cast<unsigned char>(c)));
            if (key == "ESCAPE")           lua_pushstring(L, "TOGGLEGAMEMENU");
            else if (key == "PRINTSCREEN") lua_pushstring(L, "SCREENSHOT");
            else                           lua_pushnil(L);
            return 1;
        }},
                // GetQuestGreenRange() - how many levels below the player a
                // thing can be before it turns grey.
                //
                // This was left unbound on the grounds that a guessed constant
                // would mis-colour every quest and that nothing live called it.
                // The second half was wrong: TargetFrame_CheckLevel reaches it
                // through GetQuestDifficultyColor for any target the player can
                // attack, so it raised before levelText:Show() and an enemy's
                // level never appeared. A friendly target took the other branch
                // and showed its level, which is why it looked like enemies
                // specifically.
                //
                // No guess needed either. AzerothCore's Formulas.h gives the
                // grey level exactly, and the green range is the distance from
                // the player's level down to it.
                {"GetQuestGreenRange", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int lvl = gh ? static_cast<int>(gh->getPlayerLevel()) : 1;
            int gray;
            if (lvl <= 5)       gray = 0;
            else if (lvl <= 39) gray = lvl - 5 - lvl / 10;
            else if (lvl <= 59) gray = lvl - 1 - lvl / 5;
            else                gray = lvl - 9;
            lua_pushnumber(L, lvl - gray);
            return 1;
        }},
                // Two latent raises on the escape-key path, bound as no-ops
                // because doing nothing is the right behaviour rather than a
                // placeholder for it. SetUIVisibility is reached only from
                // ToggleGameMenu's first branch, which runs when UIParent is
                // hidden - out of the way until someone hides the interface,
                // and then on the only path that brings it back.
                // ConsoleAddMessage is the sink for FrameXML's debug print.
                {"SetUIVisibility",   [](lua_State* L) -> int { (void)L; return 0; }},
                {"ConsoleAddMessage", [](lua_State* L) -> int { (void)L; return 0; }},
                // GetCurrentBindingSet() - which set SaveBindings should write.
                //
                // 1 is account-wide and 2 is per character. This client keeps
                // one set of bindings, so it is always the first. It was
                // missing rather than stubbed, and interfaceoptionspanels.lua
                // calls SaveBindings(GetCurrentBindingSet()) from three
                // separate option handlers - so changing an option would have
                // thrown rather than saved.
                {"GetCurrentBindingSet", [](lua_State* L) -> int {
            lua_pushnumber(L, 1); return 1;
        }},
                {"SaveBindings",        lua_SaveBindings},
                {"LoadBindings",        lua_LoadBindings},
                {"RunBinding",          lua_RunBinding},
                {"SetOverrideBindingClick", lua_SetOverrideBindingClick},
                {"SetOverrideBinding",      lua_SetOverrideBinding},
                {"SetOverrideBindingSpell", lua_SetOverrideBindingSpell},
                {"SetOverrideBindingMacro", lua_SetOverrideBindingMacro},
                {"SetOverrideBindingItem",  lua_SetOverrideBindingItem},
                {"ClearOverrideBindings", lua_ClearOverrideBindings},
                // Paging lives here, and the getter with it. Both were
                // defined twice - this pair against __WoweeActionBarPage and a
                // bootstrap pair against a local - so whichever won,
                // ChangeActionBarPage could move one number while
                // GetActionBarPage read the other. Removing only the getter
                // made that worse rather than better, which is what happened
                // an hour ago.
                {"GetActionBarPage", [](lua_State* L) -> int {
            lua_getglobal(L, "__WoweeActionBarPage");
            if (lua_isnil(L, -1)) { lua_pop(L, 1); lua_pushnumber(L, 1); }
            return 1;
        }},
                {"ChangeActionBarPage", [](lua_State* L) -> int {
            int page = static_cast<int>(luaL_checknumber(L, 1));
            if (page < 1) page = 1;
            if (page > 6) page = 6;
            lua_pushnumber(L, page);
            lua_setglobal(L, "__WoweeActionBarPage");
            // The number keys are handled client-side against a page of their
            // own, so it has to be told or they go on casting page one.
            if (auto* svc = getLuaServices(L); svc && svc->setActionBarPage) {
                svc->setActionBarPage(page);
            }
            // Through the engine, which delivers to both registries.
            //
            // This walked __WoweeEvents by hand and stopped there. That table
            // is where an addon's RegisterEvent lands; FrameXML registers
            // through frame:RegisterEvent, which fills __WoweeFrameEvents - so
            // the six frames that listen for this, including the one in
            // actionbutton.lua that redraws every button, were never told.
            //
            // The page number moved and nothing else did, which is exactly
            // what the arrows beside the bar looked like: they clicked, they
            // played their sound, and the icons stayed where they were.
            lua_getfield(L, LUA_REGISTRYINDEX, "wowee_lua_engine");
            auto* engine = static_cast<LuaEngine*>(lua_touserdata(L, -1));
            lua_pop(L, 1);
            if (engine) engine->fireEvent("ACTIONBAR_PAGE_CHANGED", {});
            return 0;
        }},
                // Two returns: whether there is a pet interface at all, and
                // whether it is a hunter's. PetFrame_SetHappiness reads the
                // second to decide whether to draw the happiness icon, and one
                // return left it nil.
                {"HasPetUI", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const bool has = gh && gh->hasPet();
            lua_pushboolean(L, has ? 1 : 0);
            lua_pushboolean(L, 0);   // hunter pet: not distinguished yet
            return 2;
        }},
                // Happiness, for a hunter's pet only. Nil is the honest answer
                // for everyone else and the one PetFrame_SetHappiness guards
                // for - the fallback answering with an object instead made
                // that guard pass and the branch index nothing.
                // GetMirrorTimerInfo(index) → timer, value, maxvalue, scale,
                // paused, label.
                //
                // "UNKNOWN" for a timer that is not running, which is exactly
                // what mirrortimer.lua tests for before using the rest. Absent,
                // the fallback answered with an object, that test passed, and
                // the next line divided a nil by a thousand.
                {"GetMirrorTimerInfo", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int index = static_cast<int>(luaL_optnumber(L, 1, 1)) - 1;
            // Shared with the packet handler that fires MIRROR_TIMER_START, so
            // that polling and event agree on the name MirrorTimer_Hide will
            // later be asked to match.
            const auto& kNames  = game::GameHandler::kMirrorTimerNames;
            const auto& kLabels = game::GameHandler::kMirrorTimerLabels;
            if (!gh || index < 0 || index > 2 || !gh->getMirrorTimer(index).active) {
                lua_pushstring(L, "UNKNOWN");
                lua_pushnumber(L, 0); lua_pushnumber(L, 0); lua_pushnumber(L, 0);
                lua_pushboolean(L, 0);
                lua_pushstring(L, "");
                return 6;
            }
            const auto& t = gh->getMirrorTimer(index);
            lua_pushstring(L, kNames[index]);
            lua_pushnumber(L, t.value);
            lua_pushnumber(L, t.maxValue);
            lua_pushnumber(L, t.scale);
            lua_pushboolean(L, t.paused ? 1 : 0);
            lua_pushstring(L, kLabels[index]);
            return 6;
        }},
                // GetPetHappiness() → happiness 1..3, and what it does to damage
                //
                // Both values, and that is not tidiness. The pet frame does
                // format(PET_DAMAGE_PERCENTAGE, damagePercentage) - "Causes %d%%
                // of normal damage" - the moment happiness is anything at all,
                // and %d against nil raises. Answering nil, as this did before,
                // was safe only because the line above it returns early on a nil
                // happiness; filling in the first value alone would have turned
                // a blank indicator into a dead pet frame for every hunter.
                //
                // Happiness is a power like mana, at index 4, running from
                // nothing to its own maximum. The three faces are the thirds of
                // that range, and the damage each is worth - three quarters,
                // normal, a quarter more - is the game's, not a guess.
                {"GetPetHappiness", [](lua_State* L) -> int {
            auto* pet = resolveUnit(L, "pet");
            if (!pet) { lua_pushnil(L); return 1; }
            const uint32_t maxHappiness = pet->getMaxPowerByType(4);
            if (maxHappiness == 0) { lua_pushnil(L); return 1; }

            const uint32_t happiness = pet->getPowerByType(4);
            const int level = (happiness * 3 >= maxHappiness * 2) ? 3
                            : (happiness * 3 >= maxHappiness)     ? 2
                                                                  : 1;
            static const int kDamagePercent[4] = {0, 75, 100, 125};
            lua_pushnumber(L, level);
            lua_pushnumber(L, kDamagePercent[level]);
            return 2;
        }},
                // Only referenced from commented-out code in 3.3.5, but a
                // temporary pet's timer is nil when there is no timer.
                {"GetPetTimeRemaining", [](lua_State* L) -> int {
            lua_pushnil(L);
            return 1;
        }},
                // GetPetActionSlotUsable(index) → whether the pet can use it
                //
                // Read as "if usable then draw it normally, else grey it out",
                // so an absent answer greyed out every ability the pet has. A
                // slot holding a spell is usable; an empty one has nothing to
                // grey.
                // CancelItemTempEnchantment(hand) - drop a weapon imbue
                //
                // The interface counts the hands from one and the request from
                // zero, which is the only thing between them.
                {"CancelItemTempEnchantment", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int hand = static_cast<int>(luaL_optnumber(L, 1, 0));
            if (gh && (hand == 1 || hand == 2)) {
                gh->cancelTempEnchantment(static_cast<uint8_t>(hand - 1));
            }
            return 0;
        }},
                {"GetPetActionSlotUsable", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int index = static_cast<int>(luaL_optnumber(L, 1, 0));
            if (!gh || index < 1 || index > game::GameHandler::PET_ACTION_BAR_SLOTS) {
                lua_pushboolean(L, 0);
                return 1;
            }
            const uint32_t spellId = gh->getPetActionSlot(index - 1) & 0x00FFFFFF;
            lua_pushboolean(L, spellId != 0 ? 1 : 0);
            return 1;
        }},
                // GetPetActionInfo(index) →
                //   name, subtext, texture, isToken, isActive,
                //   autoCastAllowed, autoCastEnabled
                //
                // Six of the pet bar's ten slots are not spells at all: three
                // commands (attack, follow, stay) and three stances. This read
                // every slot as a spell, so the low 24 bits - which for those
                // six are 0, 1 or 2 - were looked up as spell ids. The bar's
                // command buttons were drawn with whatever Spell.dbc has at
                // those numbers, or with "Unknown" and a question mark.
                //
                // isToken says which kind a slot is, and PetActionBar_Update
                // reads name and texture as *global names* when it is set:
                //     petActionIcon:SetTexture(_G[texture])
                // Answering a flat false meant even a correct path would have
                // been used the wrong way round.
                {"GetPetActionInfo", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            int index = static_cast<int>(luaL_checknumber(L, 1));
            if (!gh || index < 1 || index > game::GameHandler::PET_ACTION_BAR_SLOTS) {
                return luaReturnNil(L);
            }
            const uint32_t packed = gh->getPetActionSlot(index - 1);
            if (packed == 0) { return luaReturnNil(L); }

            const uint32_t action = game::pet::petActionId(packed);
            const auto type = game::pet::petActionType(packed);

            const char* tokenName = nullptr;
            const char* tokenTexture = nullptr;
            bool active = false;

            if (type == game::pet::ActionType::Command) {
                switch (action) {
                    case game::pet::kStay:
                        tokenName = "PET_ACTION_WAIT";    tokenTexture = "PET_WAIT_TEXTURE";   break;
                    case game::pet::kFollow:
                        tokenName = "PET_ACTION_FOLLOW";  tokenTexture = "PET_FOLLOW_TEXTURE"; break;
                    case game::pet::kAttack:
                        tokenName = "PET_ACTION_ATTACK";  tokenTexture = "PET_ATTACK_TEXTURE"; break;
                    case game::pet::kAbandon:
                        tokenName = "PET_DISMISS";        tokenTexture = "PET_DISMISS_TEXTURE"; break;
                    default: break;
                }
            } else if (type == game::pet::ActionType::Reaction) {
                switch (action) {
                    case game::pet::kPassive:
                        tokenName = "PET_MODE_PASSIVE";    tokenTexture = "PET_PASSIVE_TEXTURE";    break;
                    case game::pet::kDefensive:
                        tokenName = "PET_MODE_DEFENSIVE";  tokenTexture = "PET_DEFENSIVE_TEXTURE";  break;
                    case game::pet::kAggressive:
                        tokenName = "PET_MODE_AGGRESSIVE"; tokenTexture = "PET_AGGRESSIVE_TEXTURE"; break;
                    default: break;
                }
                // The stance the pet is in is the one drawn as chosen.
                active = (action == gh->getPetReact());
            }

            if (tokenName) {
                lua_pushstring(L, tokenName);       // 1: name - a global's name
                lua_pushstring(L, "");              // 2: subtext
                lua_pushstring(L, tokenTexture);    // 3: texture - likewise
                lua_pushboolean(L, 1);              // 4: isToken
                lua_pushboolean(L, active ? 1 : 0); // 5: isActive
                // A command or a stance has nothing to cast automatically, and
                // saying otherwise puts the autocast ring on all six of them.
                lua_pushboolean(L, 0);              // 6: autoCastAllowed
                lua_pushboolean(L, 0);              // 7: autoCastEnabled
                return 7;
            }

            const uint32_t spellId = action;
            if (spellId == 0) { return luaReturnNil(L); }
            const std::string& name = gh->getSpellName(spellId);
            const std::string iconPath = gh->getSpellIconPath(spellId);
            lua_pushstring(L, name.empty() ? "Unknown" : name.c_str());
            lua_pushstring(L, "");
            lua_pushstring(L, iconPath.empty() ? "Interface\\Icons\\INV_Misc_QuestionMark"
                                               : iconPath.c_str());
            lua_pushboolean(L, 0);                                      // isToken
            // A pet spell is never "active" - that is the checked ring, which
            // belongs to the stance. Reading the autocast bits as active put
            // it on every spell the pet knows.
            lua_pushboolean(L, 0);                                      // isActive
            lua_pushboolean(L, 1);                                      // autoCastAllowed
            lua_pushboolean(L, gh->isPetSpellAutocast(spellId) ? 1 : 0);
            return 7;
        }},
                // IsPetAttackAction(index) - which slot is the attack command,
                // so the bar can flash it while the pet is on a target.
                {"IsPetAttackAction", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int index = static_cast<int>(luaL_optnumber(L, 1, 0));
            bool isAttack = false;
            if (gh && index >= 1 && index <= game::GameHandler::PET_ACTION_BAR_SLOTS) {
                const uint32_t packed = gh->getPetActionSlot(index - 1);
                isAttack = game::pet::petActionType(packed) == game::pet::ActionType::Command &&
                           game::pet::petActionId(packed) == game::pet::kAttack;
            }
            lua_pushboolean(L, isAttack ? 1 : 0);
            return 1;
        }},
                // GetPetActionCooldown(index) → start, duration, enable
                //
                // A flat zero, so a pet ability on cooldown was drawn ready and
                // clickable. The client has tracked pet spell cooldowns all
                // along - they arrive on the same packet as the player's.
                //
                // Only the remaining time is kept, not the original duration,
                // so the cooldown is reported as starting now and lasting what
                // is left. That draws exactly the remaining arc, which is the
                // part anyone reads, and it is what the item cooldown beside it
                // does for the same reason.
                {"GetPetActionCooldown", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int index = static_cast<int>(luaL_optnumber(L, 1, 0));
            // Enable is one, not zero: zero switches the sweep off entirely,
            // and all three go straight into CooldownFrame_SetTimer.
            if (!gh || index < 1 || index > game::GameHandler::PET_ACTION_BAR_SLOTS) {
                lua_pushnumber(L, 0); lua_pushnumber(L, 0); lua_pushnumber(L, 1);
                return 3;
            }
            const uint32_t packed = gh->getPetActionSlot(index - 1);
            // A command or a stance has no cooldown of its own.
            const float remaining = game::pet::isPetSpellAction(packed)
                ? gh->getSpellCooldown(game::pet::petActionId(packed)) : 0.0f;
            if (remaining <= 0.01f) {
                lua_pushnumber(L, 0); lua_pushnumber(L, 0); lua_pushnumber(L, 1);
                return 3;
            }
            lua_pushnumber(L, luaGetTimeNow());
            lua_pushnumber(L, remaining);
            lua_pushnumber(L, 1);
            return 3;
        }},
                {"PetAttack", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (gh && gh->hasPet() && gh->hasTarget())
                gh->sendPetAction(
                    game::pet::packPetAction(game::pet::ActionType::Command, game::pet::kAttack),
                    gh->getTargetGuid());
            return 0;
        }},
                {"PetFollow", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (gh && gh->hasPet())
                gh->sendPetAction(
                    game::pet::packPetAction(game::pet::ActionType::Command, game::pet::kFollow), 0);
            return 0;
        }},
                {"PetWait", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (gh && gh->hasPet())
                gh->sendPetAction(
                    game::pet::packPetAction(game::pet::ActionType::Command, game::pet::kStay), 0);
            return 0;
        }},
                {"PetPassiveMode", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (gh && gh->hasPet())
                gh->sendPetAction(
                    game::pet::packPetAction(game::pet::ActionType::Reaction, game::pet::kPassive), 0);
            return 0;
        }},
                {"CastPetAction", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            int index = static_cast<int>(luaL_checknumber(L, 1));
            if (!gh || !gh->hasPet() || index < 1 || index > game::GameHandler::PET_ACTION_BAR_SLOTS) return 0;
            uint32_t packed = gh->getPetActionSlot(index - 1);
            uint32_t spellId = packed & 0x00FFFFFF;
            if (spellId != 0) {
                // CastPetAction(action, unit) - the unit is what a click-cast
                // binding on a unit frame passes, and dropping it sent the pet
                // at whatever was targeted instead of what was clicked. The
                // same fault CastSpellByID had, two files over.
                uint64_t target = 0;
                if (const char* uid = luaL_optstring(L, 2, nullptr)) {
                    std::string u(uid);
                    toLowerInPlace(u);
                    target = resolveUnitGuid(gh, u);
                }
                if (target == 0)
                    target = gh->hasTarget() ? gh->getTargetGuid() : gh->getPetGuid();
                gh->sendPetAction(packed, target);
            }
            return 0;
        }},
                {"TogglePetAutocast", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            int index = static_cast<int>(luaL_checknumber(L, 1));
            if (!gh || !gh->hasPet() || index < 1 || index > game::GameHandler::PET_ACTION_BAR_SLOTS) return 0;
            uint32_t packed = gh->getPetActionSlot(index - 1);
            uint32_t spellId = packed & 0x00FFFFFF;
            if (spellId != 0) gh->togglePetSpellAutocast(spellId);
            return 0;
        }},
                {"PetDismiss", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (gh && gh->hasPet())
                gh->sendPetAction(
                    game::pet::packPetAction(game::pet::ActionType::Command, game::pet::kAbandon), 0);
            return 0;
        }},
                // The other end of the ABANDON_PET dialog, which the pet's
                // unit menu opens. It was built, shown, and raised on accept.
                //
                // Not the same as PetDismiss above, which sends the pet away
                // and can be undone by calling it back. Abandon is permanent,
                // which is what the dialog in front of it is for.
                {"PetAbandon", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (gh && gh->hasPet()) gh->abandonPet();
            return 0;
        }},
                // CONFIRM_BUY_STABLE_SLOT's accept, from the stable panel.
                {"BuyStableSlot", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L)) gh->buyStableSlot();
            return 0;
        }},
                // VOTE_BOOT_PLAYER's two buttons. The dialog is shown from
                // LFG_BOOT_PROPOSAL_UPDATE, which is fired, so a vote to kick
                // reached the player and neither answer went anywhere.
                //
                // Both buttons matter here: declining is a vote, not a
                // dismissal, and OnCancel sends it.
                {"SetLFGBootVote", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L)) gh->lfgSetBootVote(lua_toboolean(L, 1) != 0);
            return 0;
        }},
                // CONFIRM_RESET_INSTANCES' accept, from /resetinstances and
                // the party menu.
                {"ResetInstances", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L)) gh->resetInstances();
            return 0;
        }},
                // UNLEARN_SKILL's accept, from the skill panel's unlearn
                // button. The skill line comes from the button's own row.
                // Takes a skill id here, and FrameXML's one caller passes it
                // whatever it was handed as the popup's data - which in the
                // original client is the skills tab's row index, not an id.
                // Nothing shows UNLEARN_SKILL in this build (GetSkillLineInfo
                // answers isAbandonable false, so no row offers to unlearn), so
                // the two never meet; if the popup is ever raised, whichever
                // side passes the index has to resolve it through skillRows()
                // first, or this unlearns the skill whose id happens to equal a
                // row number.
                {"AbandonSkill", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const auto skillId = static_cast<uint32_t>(luaL_optnumber(L, 1, 0));
            if (gh && skillId) gh->unlearnSkill(skillId);
            return 0;
        }},
                {"IsPetAttackActive", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushboolean(L, gh && gh->getPetCommand() == 2 ? 1 : 0); // 2=attack
            return 1;
        }},
                {"PetDefensiveMode", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (gh && gh->hasPet())
                gh->sendPetAction(
                    game::pet::packPetAction(game::pet::ActionType::Reaction, game::pet::kDefensive), 0);
            return 0;
        }},
                // The third stance, which was a no-op in another file entirely
                // - so a hunter could set a pet passive or defensive and not
                // aggressive, and the button for it did nothing.
                {"PetAggressiveMode", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (gh && gh->hasPet())
                gh->sendPetAction(
                    game::pet::packPetAction(game::pet::ActionType::Reaction, game::pet::kAggressive), 0);
            return 0;
        }},
    };
    for (const auto& [name, func] : api) {
        lua_pushcfunction(L, func);
        lua_setglobal(L, name);
    }

    // The cursor, for this client's own windows to read. They keep a separate
    // held item of their own, so a drag out of a handed-over bag into a window
    // that is still this client's - the bank, the mailbox, a trade - had no way
    // to know anything had been picked up.
    //
    // Registered here rather than mirrored into the ui layer because this is
    // where the cursor is. Re-registered on every reload, which rebuilds the
    // Lua state, so the captured pointer is the current one; LuaEngine::shutdown
    // takes the bridge back down again.
    ui::frameXmlSetCursorBridge(
        [](uint8_t& bag, uint8_t& slot) { return cursorWireSlot(bag, slot); },
        [L]() { clearCursorItem(L); });
}

} // namespace wowee::addons
