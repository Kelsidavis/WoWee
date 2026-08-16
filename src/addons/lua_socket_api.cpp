// lua_socket_api.cpp - putting gems into an item's sockets.
//
// This client has never had a socketing window. It parses socket colours and
// draws them on a tooltip, it has a handler for the server's reply, and there
// was nothing in between: no way to open an item, no way to place a gem, and
// nothing anywhere that sent CMSG_SOCKET_GEMS. FrameXML ships the whole panel -
// Blizzard_ItemSocketingUI, plus the right-click entries in the paperdoll and
// the bags that open it - and wanted six calls this client did not answer.
// tools/framexml_nil_arithmetic.py had been reporting them as the one live
// carrier left: ItemSocketingFrame_Update, reached from UIParent_OnEvent.
//
// What is real here and what is not:
//
//   * The sockets are real, read from the item template this client already
//     parses. So is the gem in a socket: the item's enchantment fields carry an
//     enchantment id, and SpellItemEnchantment names the gem it came out of.
//   * Whether a gem *matches* its socket is real, out of GemProperties.dbc -
//     the colour mask of the gem against the colour mask of the socket.
//   * The refund and bound-tradeable windows are not modelled. Both answer nil,
//     which is what GetContainerItemPurchaseInfo already answers and for the
//     same reason: the per-item timer behind them is server state this client
//     is never sent. Answering yes would promise a refund that does not exist.
#include "addons/lua_api_helpers.hpp"
#include "game/inventory_slots.hpp"
#include "game/game_utils.hpp"
#include "addons/lua_engine.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/dbc_layout.hpp"
#include "ui/framexml_takeover.hpp"

#include <array>
#include <string>

namespace wowee::addons {

namespace {

/// The four socket colours FrameXML knows, keyed by the mask an item carries.
/// GEM_TYPE_INFO in blizzard_itemsocketingui.lua is indexed by exactly these
/// strings, and an unknown one indexes nothing and takes the panel down.
const char* socketColorName(uint32_t mask) {
    if (mask & 0x1) return "Meta";
    if (mask & 0x2) return "Red";
    if (mask & 0x4) return "Yellow";
    if (mask & 0x8) return "Blue";
    return "";
}

/// A gem's own colour mask, from GemProperties.dbc.
///
/// An item's socket has a colour; a gem does not. What a gem has is a
/// GemProperties id - the field straight after socketBonus in the item query,
/// which this client kept nothing of until the socket block was fixed - and
/// that row's Type is the mask. The compound colours matter: an orange gem is
/// 6, red|yellow, and fits either a red or a yellow socket.
uint32_t gemColorMask(game::GameHandler* gh, uint32_t gemItemId) {
    if (!gh || gemItemId == 0) return 0;
    const auto* info = gh->getItemInfo(gemItemId);
    if (!info || info->gemProperties == 0) return 0;

    auto* am = gh->services().assetManager;
    if (!am || !am->isInitialized()) return 0;
    auto dbc = am->loadDBC("GemProperties.dbc");
    if (!dbc || !dbc->isLoaded()) return 0;
    const int32_t row = dbc->findRecordById(info->gemProperties);
    if (row < 0) return 0;
    // ID, Enchant_Id, MaxCountInv, MaxCountItem, Type - five columns in every
    // shape that has gems at all, so the last one is the colour.
    if (dbc->getFieldCount() < 5) return 0;
    return dbc->getUInt32(static_cast<uint32_t>(row), 4);
}

/// The template of the item whose sockets are on screen.
const game::ItemQueryResponseData* socketItemInfo(game::GameHandler* gh) {
    if (!gh || !gh->isSocketingOpen()) return nullptr;
    const uint32_t itemId = gh->getSocketItemId();
    if (itemId == 0) return nullptr;
    const auto* info = gh->getItemInfo(itemId);
    return (info && info->valid) ? info : nullptr;
}

/// The colour mask of socket `index`, or zero when there is no such socket.
uint32_t socketMaskAt(game::GameHandler* gh, int index) {
    const auto* info = socketItemInfo(gh);
    if (!info || index < 0 || index > 2) return 0;
    return info->socketColor[index];
}

/// name, icon, gemMatchesSocket for a gem item in a given socket - the three
/// returns both socket-info calls share. Pushes nothing and returns 0 when
/// there is no gem, which is how FrameXML asks "is this socket filled".
int pushGemInfo(lua_State* L, game::GameHandler* gh, uint32_t gemItemId, int socketIndex) {
    // Three nils rather than nothing when the socket is empty. Lua reads the
    // two the same way and every caller here guards on the first, but a
    // binding that answers fewer values than the interface unpacks is a shape
    // tools/framexml_short_returns.py counts, and it is right to: the ones that
    // are deliberate are indistinguishable from the ones that are a bug.
    auto pushEmpty = [L]() { lua_pushnil(L); lua_pushnil(L); lua_pushnil(L); return 3; };
    if (gemItemId == 0) return pushEmpty();
    const auto* gem = gh->getItemInfo(gemItemId);
    if (!gem || !gem->valid) {
        // Not queried yet. Ask, and answer nothing this frame - the panel
        // redraws on the next SOCKET_INFO_UPDATE with the name filled in.
        gh->ensureItemInfo(gemItemId);
        return pushEmpty();
    }
    lua_pushstring(L, gem->name.c_str());
    lua_pushstring(L, gh->getItemIconPath(gem->displayInfoId).c_str());
    const uint32_t socketMask = socketMaskAt(gh, socketIndex);
    const uint32_t gemMask = gemColorMask(gh, gemItemId);
    // A meta socket takes only a meta gem, and a meta gem fits nothing else.
    // Every other colour is a mask test: an orange gem is red|yellow and fits
    // either. Both sides of that are already masks, so one AND says it.
    lua_pushboolean(L, (socketMask & gemMask) != 0 ? 1 : 0);
    return 3;
}

/// An item link for the gem in a socket, or nothing when the socket is empty.
///
/// The panel's own tooltips work off the socket index, but a link is what a
/// shift-click puts in the chat box and what an addon hands around. Reachable
/// at all only since chat began drawing links.
int pushGemLink(lua_State* L, game::GameHandler* gh, uint32_t gemItemId) {
    if (!gh || gemItemId == 0) return 0;
    const auto* gem = gh->getItemInfo(gemItemId);
    if (!gem || !gem->valid) {
        gh->ensureItemInfo(gemItemId);
        return 0;
    }
    lua_pushstring(L, game::buildItemLink(gemItemId, gem->quality, gem->name).c_str());
    return 1;
}

/// GetExistingSocketLink(index) - the gem already in the socket.
int lua_GetExistingSocketLink(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int index = static_cast<int>(luaL_optnumber(L, 1, 1)) - 1;
    if (!gh || !gh->isSocketingOpen() || index < 0 || index > 2) return 0;
    const auto enchants = gh->getItemSocketEnchantIds(gh->getSocketItemGuid());
    return pushGemLink(L, gh, gh->getEnchantGemItem(enchants[static_cast<size_t>(index)]));
}

/// GetNewSocketLink(index) - the gem waiting to go in.
int lua_GetNewSocketLink(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int index = static_cast<int>(luaL_optnumber(L, 1, 1)) - 1;
    if (!gh || !gh->isSocketingOpen() || index < 0 || index > 2) return 0;
    return pushGemLink(L, gh, gh->getSocketPendingGemItemId(index));
}

/// SocketInventoryItem(invSlot) - the paperdoll's right-click entry.
int lua_SocketInventoryItem(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int slotId = static_cast<int>(luaL_optnumber(L, 1, 0));
    if (!gh || slotId < 1 || slotId > 19) return 0;
    gh->openSocketing(gh->getEquipSlotGuid(slotId - 1));
    return 0;
}

/// SocketContainerItem(bag, slot) - the same entry on a bag item.
int lua_SocketContainerItem(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int bag = static_cast<int>(luaL_optnumber(L, 1, 0));
    const int slot = static_cast<int>(luaL_optnumber(L, 2, 0));
    if (!gh) return 0;
    gh->openSocketing(containerSlotGuid(gh, bag, slot));
    return 0;
}

int lua_GetNumSockets(lua_State* L) {
    auto* gh = getGameHandler(L);
    const auto* info = socketItemInfo(gh);
    int count = 0;
    if (info) {
        // Count sockets, not filled ones, and stop at the first empty colour:
        // the template lists them in order and a gap would mean a socket that
        // is not there.
        for (int i = 0; i < 3 && info->socketColor[i] != 0; ++i) ++count;
    }
    lua_pushnumber(L, count);
    return 1;
}

/// GetSocketTypes(index) → the colour name, which indexes GEM_TYPE_INFO.
int lua_GetSocketTypes(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int index = static_cast<int>(luaL_optnumber(L, 1, 1)) - 1;
    lua_pushstring(L, socketColorName(socketMaskAt(gh, index)));
    return 1;
}

/// GetExistingSocketInfo(index) → the gem already in the socket.
///
/// Which is not in the item template: a gem an owner put in lives in the item's
/// enchantment fields, three of which are sockets, and each holds an
/// enchantment id rather than an item. SpellItemEnchantment.Src_ItemID is the
/// way back to the gem.
int lua_GetExistingSocketInfo(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int index = static_cast<int>(luaL_optnumber(L, 1, 1)) - 1;
    if (!gh || !gh->isSocketingOpen() || index < 0 || index > 2) {
        return pushGemInfo(L, gh, 0, index);
    }
    const auto enchants = gh->getItemSocketEnchantIds(gh->getSocketItemGuid());
    return pushGemInfo(L, gh, gh->getEnchantGemItem(enchants[index]), index);
}

/// GetNewSocketInfo(index) → the gem the player has put in but not committed.
int lua_GetNewSocketInfo(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int index = static_cast<int>(luaL_optnumber(L, 1, 1)) - 1;
    if (!gh || !gh->isSocketingOpen() || index < 0 || index > 2) {
        return pushGemInfo(L, gh, 0, index);
    }
    return pushGemInfo(L, gh, gh->getSocketPendingGemItemId(index), index);
}

/// GetSocketItemInfo() → name, icon, quality of the item being socketed. The
/// panel's portrait and title.
int lua_GetSocketItemInfo(lua_State* L) {
    auto* gh = getGameHandler(L);
    const auto* info = socketItemInfo(gh);
    if (!info) { lua_pushnil(L); lua_pushnil(L); lua_pushnil(L); return 3; }
    lua_pushstring(L, info->name.c_str());
    lua_pushstring(L, gh->getItemIconPath(info->displayInfoId).c_str());
    lua_pushnumber(L, info->quality);
    return 3;
}

/// ClickSocketButton(index) - put down what is carried, or take back what is
/// in the socket when nothing is.
///
/// Only a *pending* gem comes back out. One already socketed is in the item on
/// the server and cannot be removed by clicking it; the panel's own artwork
/// says so by drawing closed brackets around it.
int lua_ClickSocketButton(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int index = static_cast<int>(luaL_optnumber(L, 1, 1)) - 1;
    if (!gh || !gh->isSocketingOpen() || index < 0 || index > 2) return 0;

    uint8_t bag = 0, slot = 0;
    if (!wowee::ui::frameXmlCursorWireSlot(bag, slot)) {
        gh->setSocketGem(index, 0, 0);
        return 0;
    }

    // The cursor speaks in wire (container, slot) pairs; the request speaks in
    // guids. Backpack slots arrive as 0xFF plus a wire slot, worn bags as their
    // container number.
    uint64_t gemGuid = 0;
    if (bag == 0xFF) {
        const int index0 = static_cast<int>(slot) - game::slots::backpackWireSlot(0);
        gemGuid = gh->getBackpackItemGuid(index0);
    } else {
        for (int b = 0; b < game::Inventory::NUM_BAG_SLOTS; ++b) {
            if (bag != static_cast<uint8_t>(game::slots::wornBagContainer(b))) continue;
            gemGuid = gh->getBagItemGuid(b, slot);
            break;
        }
    }
    if (gemGuid == 0) return 0;

    const uint32_t gemItemId = gh->getItemEntryByGuid(gemGuid);
    // Only a gem goes in a socket. Without this any item on the cursor could be
    // dropped into one, and the request would be refused by the server with
    // nothing on screen to say why.
    const auto* gem = gh->getItemInfo(gemItemId);
    if (!gem || gem->gemProperties == 0) return 0;

    if (gh->setSocketGem(index, gemGuid, gemItemId)) {
        wowee::ui::frameXmlPutCursorDown();
    }
    return 0;
}

int lua_AcceptSockets(lua_State* L) {
    if (auto* gh = getGameHandler(L)) gh->acceptSockets();
    return 0;
}

int lua_CloseSocketInfo(lua_State* L) {
    if (auto* gh = getGameHandler(L)) gh->closeSocketing();
    return 0;
}

/// The refund and bound-tradeable windows, which this client is not sent.
///
/// Both gate a warning about socketing an item that could still be handed back.
/// nil means "no window", which is true here - and the honest answer, because
/// claiming one would have the panel warn about a refund that cannot happen.
int lua_GetSocketItemRefundable(lua_State* L) {
    lua_pushnil(L);
    return 1;
}

int lua_GetSocketItemBoundTradeable(lua_State* L) {
    lua_pushnil(L);
    return 1;
}

}  // namespace

void registerSocketLuaAPI(lua_State* L) {
    static const luaL_Reg fns[] = {
        {"SocketInventoryItem",        lua_SocketInventoryItem},
        {"SocketContainerItem",        lua_SocketContainerItem},
        {"GetNumSockets",              lua_GetNumSockets},
        {"GetSocketTypes",             lua_GetSocketTypes},
        {"GetExistingSocketInfo",      lua_GetExistingSocketInfo},
        {"GetExistingSocketLink",      lua_GetExistingSocketLink},
        {"GetNewSocketLink",           lua_GetNewSocketLink},
        {"GetNewSocketInfo",           lua_GetNewSocketInfo},
        {"GetSocketItemInfo",          lua_GetSocketItemInfo},
        {"GetSocketItemRefundable",    lua_GetSocketItemRefundable},
        {"GetSocketItemBoundTradeable", lua_GetSocketItemBoundTradeable},
        {"ClickSocketButton",          lua_ClickSocketButton},
        {"AcceptSockets",              lua_AcceptSockets},
        {"CloseSocketInfo",            lua_CloseSocketInfo},
        {.name = nullptr, .func = nullptr},
    };
    for (const luaL_Reg* fn = fns; fn->name; ++fn) {
        lua_pushcfunction(L, fn->func);
        lua_setglobal(L, fn->name);
    }
}

}  // namespace wowee::addons
