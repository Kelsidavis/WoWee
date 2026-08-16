// lua_inventory_api.cpp - Items, containers, merchant, loot, equipment, trading, auction, and mail Lua API bindings.
// Extracted from lua_engine.cpp as part of §5.1 (Tame LuaEngine).
#include "game/item_text.hpp"
#include "addons/lua_api_helpers.hpp"
#include "game/inventory_slots.hpp"
#include "game/game_utils.hpp"
#include "game/auction_filters.hpp"
#include "ui/framexml_takeover.hpp"
#include "core/logger.hpp"
#include "core/config_paths.hpp"
#include <array>
#include <fstream>
#include <filesystem>
#include <ranges>
#include <sstream>
#include <set>
#include <string_view>

namespace wowee::addons {

/// What each side has staked in the open trade.
///
/// These answered zero alongside the cursor, on the reasoning that there is no
/// trade open when the money frame first reads them. That much is true, and it
/// is why they must answer a number - but it is only true at load. The client
/// tracks both amounts for the whole trade, and answering zero throughout meant
/// the trade window showed each side offering nothing however much gold was
/// actually on the table. Someone would accept a trade believing no gold was
/// coming, or that none was going.
static int lua_GetPlayerTradeMoney(lua_State* L) {
    auto* gh = getGameHandler(L);
    lua_pushnumber(L, gh ? static_cast<double>(gh->getMyTradeGold()) : 0.0);
    return 1;
}

static int lua_GetTargetTradeMoney(lua_State* L) {
    auto* gh = getGameHandler(L);
    lua_pushnumber(L, gh ? static_cast<double>(gh->getPeerTradeGold()) : 0.0);
    return 1;
}

/// Prices FrameXML reads straight into a money frame at load, before any
/// server has told us anything. Nil is not an option there: TabardFrame does
/// MoneyFrame_Update(frame, GetTabardCreationCost()) in its OnLoad, and the
/// update divides that by the copper-per-gold constants immediately.
static int lua_GetTabardCreationCost(lua_State* L) {
    lua_pushnumber(L, 100000.0);   // ten gold
    return 1;
}

static int lua_GetSendMailPrice(lua_State* L) {
    lua_pushnumber(L, 30.0);
    return 1;
}

/// Uncommon, which is the default a fresh group starts on. Concatenated
/// straight into a global name - "ITEM_QUALITY" .. threshold .. "_DESC" - so
/// it has to be a number rather than nothing.
/// GetLootThreshold() → the quality at which group loot rules kick in.
///
/// It answered a constant uncommon, which is only the default. The party data
/// carries the real threshold and GetLootMethod beside it has been reading its
/// half of the same packet all along - so the loot dropdown showed uncommon
/// for a group that had set anything else, and setting it appeared not to
/// take.
///
/// Uncommon when there is nothing to read: a threshold of zero is what an
/// ungrouped player has, and the dropdown has no entry for it.
static int lua_GetLootThreshold(lua_State* L) {
    auto* gh = getGameHandler(L);
    const uint8_t t = gh ? gh->getPartyData().lootThreshold : 0;
    lua_pushnumber(L, t > 0 ? t : 2);
    return 1;
}

static int lua_GetMoney(lua_State* L) {
    auto* gh = getGameHandler(L);
    lua_pushnumber(L, gh ? static_cast<double>(gh->getMoneyCopper()) : 0.0);
    return 1;
}

// --- Merchant/Vendor API ---


// ── Bags ───────────────────────────────────────────────────────────────────

/// GetInventoryAlertStatus(index) → 0 for undamaged, 1 low, 2 broken.
///
/// The comment that used to sit here said durability was not tracked. It is,
/// per equipped item, and GetInventoryItemDurability has been answering from
/// the same two fields all along - so the armoured-figure warning never lit
/// up whatever state the gear was in.
///
/// The index is durabilityframe.lua's own INVENTORY_ALERT_STATUS_SLOTS
/// ordering, which is neither the equipment slot order nor a contiguous run
/// of it: head, shoulders, chest, waist, legs, feet, wrists, hands, then the
/// three weapon slots.
static int lua_GetInventoryAlertStatus(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int index = static_cast<int>(luaL_optnumber(L, 1, 0));
    static const game::EquipSlot kAlertSlots[] = {
        game::EquipSlot::HEAD,   game::EquipSlot::SHOULDERS, game::EquipSlot::CHEST,
        game::EquipSlot::WAIST,  game::EquipSlot::LEGS,      game::EquipSlot::FEET,
        game::EquipSlot::WRISTS, game::EquipSlot::HANDS,     game::EquipSlot::MAIN_HAND,
        game::EquipSlot::OFF_HAND, game::EquipSlot::RANGED,
    };
    constexpr int kCount = static_cast<int>(sizeof(kAlertSlots) / sizeof(kAlertSlots[0]));
    if (!gh || index < 1 || index > kCount) { lua_pushnumber(L, 0); return 1; }
    const auto& sl = gh->getInventory().getEquipSlot(kAlertSlots[index - 1]);
    int status = 0;
    if (!sl.empty() && sl.item.maxDurability > 0) {
        // WoW's own thresholds: broken at nothing left, low at a fifth.
        if (sl.item.curDurability == 0) status = 2;
        else if (sl.item.curDurability * 5 <= sl.item.maxDurability) status = 1;
    }
    lua_pushnumber(L, status);
    return 1;
}

// GetBankSlotCost(slotsOwned) → what the next bank bag slot costs, in copper.
//
// The bank compares it against the player's money the line after it asks -
//
//     local cost = GetBankSlotCost(numSlots);
//     if( GetMoney() >= cost ) then
//
// - so nil is a comparison against nothing and takes the frame down as it
// opens. Only reached while the bank window is handed over, since the events
// that lead here are registered in its OnShow and a suppressed frame never
// runs one, but that is the case this branch exists to make work.
//
// The prices are the game's own fixed schedule for the seven buyable slots,
// not a guess and not something the server quotes: ten silver, then a gold,
// then ten, twenty-five, fifty, a hundred and two hundred and fifty. Past the
// last one there is nothing left to sell, and zero is what the real client
// answers there.
static int lua_GetBankSlotCost(lua_State* L) {
    static constexpr uint32_t kSlotPrices[] = {
        1'000, 10'000, 100'000, 250'000, 500'000, 1'000'000, 2'500'000,
    };
    constexpr int kNumBuyable = static_cast<int>(std::size(kSlotPrices));
    const int owned = static_cast<int>(luaL_optnumber(L, 1, 0));
    const bool haveAll = owned < 0 || owned >= kNumBuyable;
    lua_pushnumber(L, haveAll ? 0.0 : static_cast<double>(kSlotPrices[owned]));
    return 1;
}

// GetNumBankSlots() → how many bank bag slots are bought, and whether that is
// all of them.
//
// The count comes off the player's own update field and this client has had it
// all along - the bank window draws its bag row from exactly this number. Only
// the binding was missing, so the interface fell through to the stub that
// answers zero for anything named GetNum*, and zero is not a harmless wrong
// answer here: bankframe.lua tints every bag button past numSlots red for
// "purchase", so a player with all seven bought was shown seven for sale.
//
// The second return is WoW's own convention - 1 when there is nothing left to
// buy, nil rather than false otherwise, which is the same thing to the `if`
// that reads it.
static int lua_GetNumBankSlots(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int owned = gh ? static_cast<int>(gh->getInventory().getPurchasedBankBagSlots()) : 0;
    // Seven buyable slots, the same schedule GetBankSlotCost prices.
    constexpr int kNumBuyable = 7;
    lua_pushnumber(L, owned);
    if (owned >= kNumBuyable) lua_pushnumber(L, 1);
    else lua_pushnil(L);
    return 2;
}

/// GetContainerItemCooldown(bag, slot) → start, duration, enable.
///
/// It answered zero for everything, so a potion or a trinket in the bags never
/// drew its cooldown sweep. The bags are handed over, so FrameXML's is the only
/// one drawing them.
static int lua_GetContainerItemCooldown(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int bag = static_cast<int>(luaL_optnumber(L, 1, -1));
    const int slot = static_cast<int>(luaL_optnumber(L, 2, 0));
    double start = 0.0, duration = 0.0;
    if (gh && slot >= 1) {
        const auto& inv = gh->getInventory();
        // Through the shared resolver, so the bank and the keyring answer too:
        // BankFrame_UpdateCooldown asks this for every one of its slots.
        const game::ItemSlot* s = containerItemSlot(inv, bag, slot);
        if (s && !s->empty()) itemUseCooldown(gh, s->item.itemId, start, duration);
    }
    lua_pushnumber(L, start);
    lua_pushnumber(L, duration);
    lua_pushnumber(L, 1);
    return 3;
}

/// GetContainerItemQuestInfo(bag, slot) → isQuestItem, questId, isActive.
/// The border a quest item draws in the bags comes from this; false is the
/// answer for an ordinary item and is what nearly every slot holds.
static int lua_GetContainerItemQuestInfo(lua_State* L) {
    lua_pushboolean(L, 0);
    lua_pushnil(L);
    lua_pushboolean(L, 0);
    return 3;
}

/// KeyRingButtonIDToInvSlotID(id) → the inventory slot a keyring button maps
/// to. The keyring holds nothing here, and the identity is the least
/// surprising mapping for code that indexes with the result.
static int lua_KeyRingButtonIDToInvSlotID(lua_State* L) {
    lua_pushnumber(L, luaL_optnumber(L, 1, 0));
    return 1;
}

/// SetPortraitToTexture(texture, path) - the rounded icon a bag or panel puts
/// in its corner. Drawn square here, since the mask that rounds it is not
/// modelled, but drawn: leaving it to the fallback left the region showing
/// whatever it last held.
///
/// The first argument is a texture or the name of one, and FrameXML uses both
/// within four lines of each other - ContainerFrame_Update passes the object
/// for an ordinary bag and the name for the keyring. Taking only the object
/// meant the keyring silently kept whatever icon it had.
static int lua_SetPortraitToTexture(lua_State* L) {
    if (lua_isstring(L, 1) && !lua_isnumber(L, 1)) {
        lua_getglobal(L, lua_tostring(L, 1));
        if (!lua_istable(L, -1)) { lua_pop(L, 1); return 0; }
        lua_replace(L, 1);
    }
    if (!lua_istable(L, 1)) return 0;
    lua_getfield(L, 1, "SetTexture");
    if (!lua_isfunction(L, -1)) { lua_pop(L, 1); return 0; }
    lua_pushvalue(L, 1);
    lua_pushvalue(L, 2);
    lua_call(L, 2, 0);
    return 0;
}

/// Bag calls that do nothing here, answered rather than left to the fallback:
/// that answers with an object, and an object called as a function raises.
static int lua_ContainerNoOp(lua_State* L) { (void)L; return 0; }

/// SpellCanTargetItem() - whether something is waiting to be applied to an item.
///
/// This client does have the notion after all: a sharpening stone, an oil, an
/// enchanting scroll and a disenchant all park the use and wait for the player
/// to pick a target. Answering a flat false is what made containerframe.lua
/// take the pickup branch instead of the targeting one on every click, so
/// there was no way to finish any of them once the bags were handed over.
static int lua_SpellCanTargetItem(lua_State* L) {
    auto* gh = getGameHandler(L);
    lua_pushboolean(L, gh && gh->isAwaitingItemTarget());
    return 1;
}

// ── Merchant: buyback and repair ───────────────────────────────────────────
//
// MerchantFrame reads all of these and this client already tracks what they
// want - items sold back, the cost to repair everything - so answering them
// with real numbers costs nothing beyond saying so.

/// GetBuybackItemInfo(index) → name, texture, price, quantity, numAvailable,
/// isUsable. The buyback list is most-recent-first, as WoW numbers it.
static int lua_GetBuybackItemInfo(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int index = static_cast<int>(luaL_checknumber(L, 1));
    if (!gh || index < 1) { return luaReturnNil(L); }
    const auto& items = gh->getBuybackItems();
    if (index > static_cast<int>(items.size())) { return luaReturnNil(L); }
    const auto& bi = items[index - 1];

    lua_pushstring(L, bi.item.name.c_str());
    std::string iconPath;
    if (bi.item.displayInfoId != 0) iconPath = gh->getItemIconPath(bi.item.displayInfoId);
    if (!iconPath.empty()) lua_pushstring(L, iconPath.c_str());
    else lua_pushnil(L);
    // What it costs to take back is what it sold for, times the stack.
    lua_pushnumber(L, static_cast<double>(bi.item.sellPrice) * bi.count);
    lua_pushnumber(L, bi.count);
    lua_pushnumber(L, 1);
    lua_pushboolean(L, 1);
    return 6;
}

static int lua_GetBuybackItemLink(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int index = static_cast<int>(luaL_checknumber(L, 1));
    if (!gh || index < 1) { return luaReturnNil(L); }
    const auto& items = gh->getBuybackItems();
    if (index > static_cast<int>(items.size())) { return luaReturnNil(L); }
    const auto& bi = items[index - 1];
    const int q = static_cast<int>(bi.item.quality);
    const std::string link = game::itemChatLink(bi.item.itemId, static_cast<uint32_t>(q), bi.item.name);
    lua_pushstring(L, link.c_str());
    return 1;
}

static int lua_BuybackItem(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int index = static_cast<int>(luaL_checknumber(L, 1));
    if (gh && index >= 1) gh->buyBackItem(static_cast<uint32_t>(index - 1));
    return 0;
}

/// GetRepairAllCost() → cost, whether it can be afforded. Both are wanted
/// together: the button greys itself out on the second.
static int lua_GetRepairAllCost(lua_State* L) {
    auto* gh = getGameHandler(L);
    const uint32_t cost = gh ? gh->estimateRepairAllCost() : 0;
    lua_pushnumber(L, cost);
    lua_pushboolean(L, gh && gh->getMoneyCopper() >= cost);
    return 2;
}

static int lua_CloseMerchant(lua_State* L) {
    if (auto* gh = getGameHandler(L)) gh->closeVendor();
    return 0;
}

/// GetMerchantItemMaxStack(index) → the largest stack that can be bought at
/// once. The item's own stack limit, which is what it means.
static int lua_GetMerchantItemMaxStack(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int index = static_cast<int>(luaL_checknumber(L, 1));
    if (!gh || index < 1) { lua_pushnumber(L, 1); return 1; }
    const auto& items = gh->getVendorItems().items;
    if (index > static_cast<int>(items.size())) { lua_pushnumber(L, 1); return 1; }
    const auto* info = gh->getItemInfo(items[index - 1].itemId);
    lua_pushnumber(L, (info && info->maxStack > 0) ? info->maxStack : 1);
    return 1;
}

/// The extended cost behind a vendor slot, or null when it is bought with coin.
///
/// The badges, marks and honour some vendors charge instead of money. Parsed
/// from the vendor inventory - which not every server sends, so the field is
/// read conditionally - and looked up through getExtendedCost.
///
/// The two lines that used to open this comment said it was not tracked and
/// that zero meant "this one costs money". Both were true once and neither
/// was true of the function they sat on, which returns the entry.
static const game::GameHandler::ExtendedCostEntry* merchantCost(lua_State* L, int index) {
    auto* gh = getGameHandler(L);
    if (!gh || index < 1) return nullptr;
    const auto& items = gh->getVendorItems().items;
    if (index > static_cast<int>(items.size())) return nullptr;
    const uint32_t costId = items[index - 1].extendedCost;
    return costId ? gh->getExtendedCost(costId) : nullptr;
}

// GetMerchantItemCostInfo(index) → honorPoints, arenaPoints, itemCount
//
// Three values, not one. The merchant frame reads all three and then tests
// `itemCount > 0` - against nil that is an error rather than a false, and it
// runs for every slot the vendor shows.
static int lua_GetMerchantItemCostInfo(lua_State* L) {
    const int index = static_cast<int>(luaL_optnumber(L, 1, 0));
    const auto* cost = merchantCost(L, index);
    if (!cost) {
        lua_pushnumber(L, 0);
        lua_pushnumber(L, 0);
        lua_pushnumber(L, 0);
        return 3;
    }
    int items = 0;
    for (int j = 0; j < 5; ++j) {
        if (cost->itemId[j] != 0 && cost->itemCount[j] != 0) ++items;
    }
    lua_pushnumber(L, cost->honorPoints);
    lua_pushnumber(L, cost->arenaPoints);
    lua_pushnumber(L, items);
    return 3;
}

// GetMerchantItemCostItem(index, i) → itemTexture, itemValue, itemLink
//
// The i-th thing a vendor wants for a slot besides coin. Counts only the filled
// entries, so the second cost is the second one shown rather than whatever sits
// in the second of five fixed fields.
static int lua_GetMerchantItemCostItem(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int index = static_cast<int>(luaL_optnumber(L, 1, 0));
    const int which = static_cast<int>(luaL_optnumber(L, 2, 0));
    const auto* cost = merchantCost(L, index);
    if (!gh || !cost || which < 1) { return luaReturnNil(L); }

    int seen = 0;
    for (int j = 0; j < 5; ++j) {
        if (cost->itemId[j] == 0 || cost->itemCount[j] == 0) continue;
        if (++seen != which) continue;

        // Asked for by id, because a cost item is often one the player has
        // never seen and so was never sent with the vendor list.
        gh->ensureItemInfo(cost->itemId[j]);
        const auto* info = gh->getItemInfo(cost->itemId[j]);
        lua_pushstring(L, info ? gh->getItemIconPath(info->displayInfoId).c_str() : "");
        lua_pushnumber(L, cost->itemCount[j]);
        if (info && !info->name.empty()) {
    const std::string link = game::itemChatLink(cost->itemId[j], static_cast<uint32_t>(info->quality), info->name);
            lua_pushstring(L, link.c_str());
        } else {
            lua_pushnil(L);
        }
        return 3;
    }
    return luaReturnNil(L);
}



static int lua_GetMerchantNumItems(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { return luaReturnZero(L); }
    lua_pushnumber(L, gh->getVendorItems().items.size());
    return 1;
}

// GetMerchantItemInfo(index) → name, texture, price, stackCount, numAvailable, isUsable
static int lua_GetMerchantItemInfo(lua_State* L) {
    auto* gh = getGameHandler(L);
    int index = static_cast<int>(luaL_checknumber(L, 1));
    if (!gh || index < 1) { return luaReturnNil(L); }
    const auto& items = gh->getVendorItems().items;
    if (index > static_cast<int>(items.size())) { return luaReturnNil(L); }
    const auto& vi = items[index - 1];
    const auto* info = gh->getItemInfo(vi.itemId);
    std::string name = info ? info->name : ("Item #" + std::to_string(vi.itemId));
    lua_pushstring(L, name.c_str());                    // name
    // texture
    std::string iconPath;
    if (info && info->displayInfoId != 0)
        iconPath = gh->getItemIconPath(info->displayInfoId);
    if (!iconPath.empty()) lua_pushstring(L, iconPath.c_str());
    else lua_pushnil(L);
    lua_pushnumber(L, vi.buyPrice);                     // price (copper)
    lua_pushnumber(L, vi.stackCount > 0 ? vi.stackCount : 1); // stackCount
    lua_pushnumber(L, vi.maxCount == -1 ? -1 : vi.maxCount);  // numAvailable (-1=unlimited)
    lua_pushboolean(L, 1);                              // isUsable
    // The extended cost - tokens, honour, arena points - which merchantframe
    // reads as `if ( extendedCost and (price <= 0) )` to decide whether to
    // show a token price instead of a coin one. It was not returned at all, so
    // an item bought with marks or emblems showed as free.
    //
    // Nil rather than zero when there is none: zero is *true* in Lua, so a
    // zero here would claim every free item had a token cost.
    if (vi.extendedCost != 0) lua_pushnumber(L, vi.extendedCost);
    else                      lua_pushnil(L);
    return 7;
}

// GetMerchantItemLink(index) → item link
static int lua_GetMerchantItemLink(lua_State* L) {
    auto* gh = getGameHandler(L);
    int index = static_cast<int>(luaL_checknumber(L, 1));
    if (!gh || index < 1) { return luaReturnNil(L); }
    const auto& items = gh->getVendorItems().items;
    if (index > static_cast<int>(items.size())) { return luaReturnNil(L); }
    const auto& vi = items[index - 1];
    const auto* info = gh->getItemInfo(vi.itemId);
    if (!info) { return luaReturnNil(L); }

    const std::string link = game::itemChatLink(vi.itemId, static_cast<uint32_t>(info->quality), info->name);
    lua_pushstring(L, link.c_str());
    return 1;
}

static int lua_CanMerchantRepair(lua_State* L) {
    auto* gh = getGameHandler(L);
    lua_pushboolean(L, gh && gh->getVendorItems().canRepair ? 1 : 0);
    return 1;
}

// UnitStat(unit, statIndex) → base, effective, posBuff, negBuff

/// An item id from either an id or a link, which is what every one of these
/// functions is documented to accept.
static uint32_t itemIdFromArg(lua_State* L, int index) {
    if (lua_isnumber(L, index)) {
        return static_cast<uint32_t>(lua_tonumber(L, index));
    }
    if (lua_isstring(L, index)) {
        const char* s = lua_tostring(L, index);
        std::string str(s ? s : "");
        const auto pos = str.find("item:");
        if (pos != std::string::npos) {
            try { return static_cast<uint32_t>(std::stoul(str.substr(pos + 5))); } catch (...) {}
        }
    }
    return 0;
}

// IsDressableItem(item) → whether the dress-up model can wear or hold it
//
// Armour and weapons only: everything else has no display slot, and the frame
// opens an empty preview for anything that answers yes.
/// __WoweeTryOn(frame, itemLinkOrId) - put an item on a model frame.
///
/// DressUpModel:TryOn(link) is the "try on" every item link and auction row
/// offers. The list belongs to the frame rather than to the application: a
/// second dressing room would have its own, and closing one is what empties it.
///
/// An item already on the same inventory slot is replaced, because trying on
/// two chests means wearing the second.
static int lua_WoweeTryOn(lua_State* L) {
    auto* gh = getGameHandler(L);
    auto* tree = getWidgetTree(L);
    if (!gh || !tree || !lua_istable(L, 1)) return 0;
    lua_getfield(L, 1, "__wid");
    const uint32_t id = static_cast<uint32_t>(lua_tointeger(L, -1));
    lua_pop(L, 1);
    auto* w = tree->get(id);
    if (!w) return 0;

    const uint32_t itemId = itemIdFromArg(L, 2);
    const auto* info = itemId ? gh->getItemInfo(itemId) : nullptr;
    if (!info || info->displayInfoId == 0) return 0;

    const uint8_t slot = static_cast<uint8_t>(info->inventoryType);
    for (auto& worn : w->tryOnItems) {
        if (worn.inventoryType == slot) {
            worn.displayInfoId = info->displayInfoId;
            return 0;
        }
    }
    w->tryOnItems.push_back({info->displayInfoId, slot});
    return 0;
}

/// __WoweeSetModelCreature(frame, displayId) - what a model frame is showing.
///
/// The companion preview and the stable's paperdoll both name a creature this
/// way. The id is written onto the frame rather than acted on here, because
/// building the model needs the offscreen view and those live in the render
/// loop; zero clears it.
static int lua_WoweeSetModelCreature(lua_State* L) {
    auto* tree = getWidgetTree(L);
    if (!tree || !lua_istable(L, 1)) return 0;
    lua_getfield(L, 1, "__wid");
    const uint32_t id = static_cast<uint32_t>(lua_tointeger(L, -1));
    lua_pop(L, 1);
    if (auto* w = tree->get(id)) {
        w->modelDisplayId = static_cast<uint32_t>(luaL_optnumber(L, 2, 0));
    }
    return 0;
}

/// __WoweeUndress(frame) - take everything tried on back off.
static int lua_WoweeUndress(lua_State* L) {
    auto* tree = getWidgetTree(L);
    if (!tree || !lua_istable(L, 1)) return 0;
    lua_getfield(L, 1, "__wid");
    const uint32_t id = static_cast<uint32_t>(lua_tointeger(L, -1));
    lua_pop(L, 1);
    if (auto* w = tree->get(id)) w->tryOnItems.clear();
    return 0;
}

static int lua_IsDressableItem(lua_State* L) {
    auto* gh = getGameHandler(L);
    const uint32_t itemId = itemIdFromArg(L, 1);
    const auto* info = (gh && itemId) ? gh->getItemInfo(itemId) : nullptr;
    // 2 is weapon and 4 is armour, as the item class is sent.
    const bool dressable = info && (info->itemClass == 2 || info->itemClass == 4);
    lua_pushboolean(L, dressable ? 1 : 0);
    return 1;
}

static int lua_GetItemInfo(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { return luaReturnNil(L); }

    const uint32_t itemId = itemIdFromArg(L, 1);
    if (itemId == 0) { return luaReturnNil(L); }

    const auto* info = gh->getItemInfo(itemId);
    if (!info) {
    // Ask the server for it rather than only reporting its absence.
    //
    // An item template arrives from the server, and until it does this
    // answered nil and left it at that - so hovering anything the client had
    // not already seen gave a name and nothing else, permanently. The real
    // client sends CMSG_ITEM_QUERY_SINGLE on exactly this miss.
    //
    // Safe to call on every miss: queryItemInfo drops the request if one is
    // already pending or the entry is cached, and does nothing out of world.
    // GameTooltip re-runs its owner's UpdateTooltip every TOOLTIP_UPDATE_TIME,
    // so the lines appear on their own once the reply lands.
        gh->queryItemInfo(itemId, 0);
        return luaReturnNil(L);
    }

    lua_pushstring(L, info->name.c_str());          // 1: name
    // Build item link with quality-colored text
    const std::string link = game::itemChatLink(itemId, static_cast<uint32_t>(info->quality), info->name);
    lua_pushstring(L, link.c_str());                         // 2: link
    lua_pushnumber(L, info->quality);                // 3: quality
    lua_pushnumber(L, info->itemLevel);              // 4: iLevel
    lua_pushnumber(L, info->requiredLevel);          // 5: requiredLevel
    // 6: class (type string) - map itemClass to display name
    {
        static constexpr const char* kItemClasses[] = {
            "Consumable", "Bag", "Weapon", "Gem", "Armor", "Reagent", "Projectile",
            "Trade Goods", "Generic", "Recipe", "Money", "Quiver", "Quest", "Key",
            "Permanent", "Miscellaneous", "Glyph"
        };
        if (info->itemClass < 17)
            lua_pushstring(L, kItemClasses[info->itemClass]);
        else
            lua_pushstring(L, "Miscellaneous");
    }
    // 7: subclass - use subclassName from ItemDef if available, else generic
    lua_pushstring(L, info->subclassName.empty() ? "" : info->subclassName.c_str());
    lua_pushnumber(L, info->maxStack > 0 ? info->maxStack : 1); // 8: maxStack
    // 9: equipSlot - WoW inventoryType to INVTYPE string
    {
        static constexpr const char* kInvTypes[] = {
            "", "INVTYPE_HEAD", "INVTYPE_NECK", "INVTYPE_SHOULDER",
            "INVTYPE_BODY", "INVTYPE_CHEST", "INVTYPE_WAIST", "INVTYPE_LEGS",
            "INVTYPE_FEET", "INVTYPE_WRIST", "INVTYPE_HAND", "INVTYPE_FINGER",
            "INVTYPE_TRINKET", "INVTYPE_WEAPON", "INVTYPE_SHIELD",
            "INVTYPE_RANGED", "INVTYPE_CLOAK", "INVTYPE_2HWEAPON",
            "INVTYPE_BAG", "INVTYPE_TABARD", "INVTYPE_ROBE",
            "INVTYPE_WEAPONMAINHAND", "INVTYPE_WEAPONOFFHAND", "INVTYPE_HOLDABLE",
            "INVTYPE_AMMO", "INVTYPE_THROWN", "INVTYPE_RANGEDRIGHT",
            "INVTYPE_QUIVER", "INVTYPE_RELIC"
        };
        uint32_t invType = info->inventoryType;
        lua_pushstring(L, invType < 29 ? kInvTypes[invType] : "");
    }
    // 10: texture (icon path from ItemDisplayInfo.dbc)
    if (info->displayInfoId != 0) {
        std::string iconPath = gh->getItemIconPath(info->displayInfoId);
        if (!iconPath.empty()) lua_pushstring(L, iconPath.c_str());
        else lua_pushnil(L);
    } else {
        lua_pushnil(L);
    }
    lua_pushnumber(L, info->sellPrice);              // 11: vendorPrice
    return 11;
}

// GetItemQualityColor(quality) → r, g, b, hex
// Quality: 0=Poor(gray), 1=Common(white), 2=Uncommon(green), 3=Rare(blue),
//          4=Epic(purple), 5=Legendary(orange), 6=Artifact(gold), 7=Heirloom(gold)
static int lua_GetItemQualityColor(lua_State* L) {
    int q = static_cast<int>(luaL_checknumber(L, 1));
    struct QC { float r, g, b; const char* hex; };
    static const QC colors[] = {
        {.r = 0.62f, .g = 0.62f, .b = 0.62f, .hex = "ff9d9d9d"}, // 0 Poor
        {.r = 1.00f, .g = 1.00f, .b = 1.00f, .hex = "ffffffff"}, // 1 Common
        {.r = 0.12f, .g = 1.00f, .b = 0.00f, .hex = "ff1eff00"}, // 2 Uncommon
        {.r = 0.00f, .g = 0.44f, .b = 0.87f, .hex = "ff0070dd"}, // 3 Rare
        {.r = 0.64f, .g = 0.21f, .b = 0.93f, .hex = "ffa335ee"}, // 4 Epic
        {.r = 1.00f, .g = 0.50f, .b = 0.00f, .hex = "ffff8000"}, // 5 Legendary
        {.r = 0.90f, .g = 0.80f, .b = 0.50f, .hex = "ffe6cc80"}, // 6 Artifact
        {.r = 0.00f, .g = 0.80f, .b = 1.00f, .hex = "ff00ccff"}, // 7 Heirloom
    };
    if (q < 0 || q > 7) q = 1;
    lua_pushnumber(L, colors[q].r);
    lua_pushnumber(L, colors[q].g);
    lua_pushnumber(L, colors[q].b);
    lua_pushstring(L, colors[q].hex);
    return 4;
}

// GetItemCount(itemId [, includeBank]) → count
// ---- Money frame ----
//
// Shared by the quest giver, the merchant, the bank, the mail frame and the
// quest tracker, so one gap here is five elements' worth. None of it showed up
// in a scan of any of those frames: they reach it through the money frame's own
// file, which they pull in rather than declare.

// GetCoinText(amount, separator) → "12g 30s 45c"
//
// Denominations with a zero count are left out, as WoW does - except when the
// whole amount is zero, which prints as copper rather than as nothing.

// Moving money with the cursor: picking an amount up, dropping it into a trade,
// a mail, a mail's cash-on-delivery box, or the guild bank.
//
// Kept for the two money types this client cannot move - the guild bank's and
// the auction's - where the gesture exists in the interface and there is
// nowhere for it to land. Trade and mail have their own now: the cursor does
// carry money, and the comment that used to sit here saying otherwise was
// written before PickupPlayerMoney and setCursorMoney were.
static int lua_MoneyCursorNoop(lua_State* L) { (void)L; return 0; }

/// The drop half of the money gesture, for the trade window.
///
/// MoneyTypeInfo["PLAYER_TRADE"].DropFunc calls this with **no argument**: the
/// amount is whatever is on the cursor, which is the whole design the cursor
/// helper describes - the frame it is dropped on reads the amount, puts it
/// where it belongs and clears the cursor. A no-op meant dragging gold onto a
/// trade did nothing, and there is no other way to offer money in this panel.
static int lua_AddTradeMoney(lua_State* L) {
    auto* gh = getGameHandler(L);
    const uint64_t held = cursorMoney();
    if (!gh || held == 0) return 0;
    gh->setTradeGold(gh->getMyTradeGold() + held);
    setCursorMoney(L, 0);
    return 0;
}

/// The pick-up half: take money back off the trade onto the cursor. Without an
/// amount, all of it - which is what the money frame passes when the whole
/// figure is dragged.
static int lua_PickupTradeMoney(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) return 0;
    const uint64_t offered = gh->getMyTradeGold();
    if (offered == 0) return 0;
    uint64_t take = static_cast<uint64_t>(luaL_optnumber(L, 1, 0));
    if (take == 0 || take > offered) take = offered;
    gh->setTradeGold(offered - take);
    setCursorMoney(L, take);
    return 0;
}

// GetContainerItemPurchaseInfo(bag, slot, isEquipped) →
//   money, honorPoints, arenaPoints, itemCount, refundSec
//
// What an item could be handed back for, and how long is left to do it.
static int lua_GetContainerItemPurchaseInfo(lua_State* L) {
    // money, honorPoints, arenaPoints, itemCount, refundSeconds.
    //
    // This answered five nils on the reading that the refund window is server
    // state this client is never sent. It is sent - as a reply to
    // CMSG_ITEM_REFUND_INFO, which nothing here had ever asked. So the first
    // call for an item asks and answers nothing, and the reply fires
    // UPDATE_INVENTORY_ALERTS, which is what brings the interface back.
    //
    // containerframe.lua treats a nil fifth return as "not refundable" and
    // stops, so answering nothing while the question is in flight is the
    // correct thing rather than a placeholder.
    auto* gh = getGameHandler(L);
    const int bag = static_cast<int>(luaL_optnumber(L, 1, 0));
    const int slot = static_cast<int>(luaL_optnumber(L, 2, 0));
    const uint64_t guid = gh ? containerSlotGuid(gh, bag, slot) : 0;
    if (!gh || guid == 0) { for (int i = 0; i < 5; ++i) lua_pushnil(L); return 5; }

    const auto* info = gh->getItemRefundInfo(guid);
    if (!info) {
        gh->requestItemRefundInfo(guid);
        for (int i = 0; i < 5; ++i) lua_pushnil(L);
        return 5;
    }

    // Two hours of played time, which is what the server counts down. Past it
    // there is no window left, and nil is how that is said.
    constexpr uint32_t kRefundWindow = 2 * 60 * 60;
    uint32_t itemCount = 0;
    for (const auto& cost : info->items) if (cost.first != 0) itemCount += cost.second;

    lua_pushnumber(L, info->money);
    lua_pushnumber(L, info->honor);
    lua_pushnumber(L, info->arena);
    lua_pushnumber(L, itemCount);
    if (info->playedSincePurchase < kRefundWindow)
        lua_pushnumber(L, kRefundWindow - info->playedSincePurchase);
    else
        lua_pushnil(L);
    return 5;
}

// GetContainerItemPurchaseItem(bag, slot, index, isEquipped) →
//   texture, quantity, link, name
//
// The items that would come back with a refund - a badge or a mark, whatever
// the vendor charged besides money. Reached only once the call above reports a
// live window, which it does now.
static int lua_GetContainerItemPurchaseItem(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int bag = static_cast<int>(luaL_optnumber(L, 1, 0));
    const int slot = static_cast<int>(luaL_optnumber(L, 2, 0));
    const int index = static_cast<int>(luaL_optnumber(L, 3, 1)) - 1;
    const uint64_t guid = gh ? containerSlotGuid(gh, bag, slot) : 0;
    const auto* info = guid ? gh->getItemRefundInfo(guid) : nullptr;
    if (!info || index < 0 || index >= 5 || info->items[static_cast<size_t>(index)].first == 0) {
        for (int i = 0; i < 4; ++i) lua_pushnil(L);
        return 4;
    }
    const uint32_t itemId = info->items[static_cast<size_t>(index)].first;
    const auto* item = gh->getItemInfo(itemId);
    if (!item || !item->valid) {
        gh->ensureItemInfo(itemId);
        for (int i = 0; i < 4; ++i) lua_pushnil(L);
        return 4;
    }
    lua_pushstring(L, gh->getItemIconPath(item->displayInfoId).c_str());
    lua_pushnumber(L, info->items[static_cast<size_t>(index)].second);
    lua_pushstring(L, game::buildItemLink(itemId, item->quality, item->name).c_str());
    lua_pushstring(L, item->name.c_str());
    return 4;
}

// ---- Currency tab (Blizzard_TokenUI) ----
//
// In 3.3.5a a currency is a CurrencyTypes.dbc row pointing at an item, and the
// amount held is that item's stack count in the bags. There is no separate
// currency store to read, which is why this is assembled here rather than
// tracked in the handler.
//
// Only currencies the player actually holds are listed. The real client lists
// every one ever earned, from PLAYER_FIELD_KNOWN_CURRENCIES, which is not
// parsed here - showing what is held is the subset that can be stated
// truthfully, and the tab was completely empty before.
namespace {
/// The guild bank item sitting in a tab and slot, or nullptr for an empty one.
///
/// Two bindings ask this, for the icon and for the hyperlink, and both had
/// their own copy of the tab rule: the tab currently open is the one the
/// server keeps up to date, and any other answers from whatever the last full
/// update left behind. Tabs and slots are 1-based from Lua and 0-based here.
const game::GuildBankItemSlot* guildBankItemAt(game::GameHandler* gh, int tab, int slot) {
    if (!gh) return nullptr;
    const auto& data = gh->getGuildBankData();

    const std::vector<game::GuildBankItemSlot>* items = nullptr;
    if (tab - 1 == data.tabId) {
        items = &data.tabItems;
    } else if (tab >= 1 && tab <= static_cast<int>(data.tabs.size())) {
        items = &data.tabs[tab - 1].items;
    }
    if (!items) return nullptr;

    for (const auto& entry : *items) {
        if (entry.slotId + 1 == slot) return &entry;
    }
    return nullptr;
}


struct CurrencyRow {
    std::string name;
    uint32_t    itemId = 0;
    uint32_t    currencyId = 0;
    uint32_t    count = 0;
    std::string icon;
};

uint32_t countItemInBags(game::GameHandler* gh, uint32_t itemId) {
    return gh->getInventory().countItem(itemId);
}

// Rebuilt per call rather than cached: the tab is opened rarely and the counts
// change with every loot, so a cache here would be one more thing to
// invalidate.
std::vector<CurrencyRow> buildCurrencyList(lua_State* L) {
    std::vector<CurrencyRow> rows;
    auto* gh = getGameHandler(L);
    if (!gh) return rows;
    for (const auto& c : gh->getCurrencyTypes()) {
        const uint32_t count = countItemInBags(gh, c.itemId);
        if (count == 0) continue;
        CurrencyRow r;
        r.currencyId = c.id;
        r.itemId     = c.itemId;
        r.count      = count;
        if (const auto* info = gh->getItemInfo(c.itemId)) r.name = info->name;
        if (r.name.empty()) r.name = "Item #" + std::to_string(c.itemId);
        rows.push_back(std::move(r));
    }
    std::sort(rows.begin(), rows.end(), [](const CurrencyRow& a, const CurrencyRow& b) {
        if (a.name != b.name) return a.name < b.name;
        return a.currencyId < b.currencyId;   // a total order, not just a tie-break
    });
    return rows;
}

}  // namespace

namespace {

/// FrameXML's name for a bank log line, which is what its own branches compare
/// against. AzerothCore's GuildBankEventLogTypes on one side, and the four
/// strings blizzard_guildbankui.lua tests for on the other.
const char* bankLogTypeName(uint8_t type) {
    switch (type) {
        case 1: case 4: return "deposit";
        case 2: case 5: return "withdraw";
        case 3: case 7: return "move";
        case 6:         return "repair";
        case 9:         return "buytab";
        default:        return "deposit";
    }
}

/// An item link for the log line, or nil when the entry carries no item -
/// which is every money line, and nil is what the format string skips.
void pushItemLinkOrNil(lua_State* L, game::GameHandler* gh, uint32_t itemId) {
    const auto* info = itemId ? gh->getItemInfo(itemId) : nullptr;
    if (!info || info->name.empty()) { lua_pushnil(L); return; }
    const uint32_t quality = info->quality < 8 ? info->quality : 1u;
    const std::string link = game::itemChatLink(itemId, quality, info->name);
    lua_pushstring(L, link.c_str());
}

/// years, months, days, hours - each counted *ago*, which is how
/// RecentTimeDate reads them: it takes the largest that is not zero and says
/// "N days ago". The server sends one number of seconds, so they are derived
/// from it rather than from a calendar.
void pushTimeAgo(lua_State* L, uint32_t secondsAgo) {
    constexpr uint32_t kHour = 3600, kDay = 24 * kHour;
    constexpr uint32_t kMonth = 30 * kDay, kYear = 365 * kDay;
    lua_pushnumber(L, secondsAgo / kYear);
    lua_pushnumber(L, (secondsAgo % kYear) / kMonth);
    lua_pushnumber(L, (secondsAgo % kMonth) / kDay);
    lua_pushnumber(L, (secondsAgo % kDay) / kHour);
}

/// Where the money log lives, matching SocialHandler's own numbering.
constexpr uint8_t kGuildBankMoneyTab = 6;

}  // namespace

uint32_t currencyListItemId(lua_State* L, int index) {
    const auto rows = buildCurrencyList(L);
    if (index < 1 || index > static_cast<int>(rows.size())) return 0;
    return rows[static_cast<size_t>(index) - 1].itemId;
}

static int lua_GetItemCount(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { return luaReturnZero(L); }
    const uint32_t itemId = static_cast<uint32_t>(luaL_checknumber(L, 1));
    lua_pushnumber(L, gh->getInventory().countItem(itemId));
    return 1;
}

// SplitContainerItem(bag, slot, count) - take part of a stack onto the cursor
//
// The interface counts containers from zero for the backpack and one to four
// for the bags, with slots starting at one. The wire counts neither way: the
// backpack is 0xFF with its slots offset past the equipment, and a bag is
// nineteen plus its index with slots from zero. The mapping is the one this
// client's own inventory uses, taken from there rather than restated.
static int lua_SplitContainerItem(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int bag = static_cast<int>(luaL_checknumber(L, 1));
    const int slot = static_cast<int>(luaL_checknumber(L, 2));
    const int count = static_cast<int>(luaL_optnumber(L, 3, 1));
    if (!gh || slot < 1 || count < 1) return 0;

    // Every container the interface can name, through the one mapping.
    // bankframe splits from BANK_CONTAINER, which this could not name before.
    if (!containerItemSlot(gh->getInventory(), bag, slot)) return 0;

    // WoW's SplitContainerItem picks the split portion up rather than moving
    // it: the stack-split dialog calls this, and the player then drops what
    // they are carrying wherever they meant it to go. This sent the split to
    // the first free bag slot instead, so a stack could only ever be broken in
    // half where it already was - the drop had nothing to carry and moving part
    // of a stack anywhere else was impossible through the interface.
    //
    // The guild bank already worked this way: the amount rides on the cursor
    // and the drop names the destination.
    pickupSplitFromContainer(L, gh, bag, slot, count);
    return 0;
}

// BankButtonIDToInvSlotID(buttonID, isBag) → the equipment slot a bank button
// stands for
//
// Arithmetic, not state: the twenty-eight general bank slots follow the
// equipment at forty, and the seven bank bag slots at sixty-eight.
static int lua_BankButtonIDToInvSlotID(lua_State* L) {
    // Both from the interface's own constants: the general slots start one past
    // the offset, and the bank bags start one past the last of them. Written as
    // the sum rather than as sixty-seven so the arithmetic is visible.
    const int buttonId = static_cast<int>(luaL_checknumber(L, 1));
    const bool isBag = lua_toboolean(L, 2) != 0;
    // The buttons count from one and so do inventory slots, so the nth button
    // is the (n-1)th wire slot, crossed back into the interface's numbering.
    const int wire = isBag ? game::slots::bankBagWireSlot(buttonId - 1)
                           : game::slots::bankGeneralWireSlot(buttonId - 1);
    lua_pushnumber(L, game::slots::toInventorySlot(wire));
    return 1;
}



// --- The mailbox ---
//
// The client had every piece of this and no way in: the inbox, each mail's
// attachments, and the requests to take money, take an item, delete and send.
// Indices are the interface's, counting from one.
namespace {

const game::MailMessage* mailAt(game::GameHandler* gh, int index) {
    if (!gh || index < 1) return nullptr;
    const auto& inbox = gh->getMailInbox();
    if (index > static_cast<int>(inbox.size())) return nullptr;
    return &inbox[index - 1];
}

const game::MailAttachment* attachmentAt(const game::MailMessage* mail, int slot) {
    if (!mail || slot < 1 || slot > static_cast<int>(mail->attachments.size())) {
        return nullptr;
    }
    return &mail->attachments[slot - 1];
}

}  // namespace

// GetInboxItem(index, slot) → name, texture, count, quality, canUse
static int lua_GetInboxItem(lua_State* L) {
    auto* gh = getGameHandler(L);
    const auto* mail = mailAt(gh, static_cast<int>(luaL_optnumber(L, 1, 0)));
    const auto* att = attachmentAt(mail, static_cast<int>(luaL_optnumber(L, 2, 1)));
    if (!att) { return luaReturnNil(L); }
    const auto* info = gh->getItemInfo(att->itemId);
    lua_pushstring(L, info ? info->name.c_str() : "");
    lua_pushstring(L, info ? gh->getItemIconPath(info->displayInfoId).c_str() : "");
    lua_pushnumber(L, att->stackCount);
    lua_pushnumber(L, info ? info->quality : 1);
    lua_pushboolean(L, 1);
    return 5;
}

// GetInboxItemLink(index, slot) → the link for a tooltip
static int lua_GetInboxItemLink(lua_State* L) {
    auto* gh = getGameHandler(L);
    const auto* mail = mailAt(gh, static_cast<int>(luaL_optnumber(L, 1, 0)));
    const auto* att = attachmentAt(mail, static_cast<int>(luaL_optnumber(L, 2, 1)));
    const auto* info = att ? gh->getItemInfo(att->itemId) : nullptr;
    if (!info || info->name.empty()) { return luaReturnNil(L); }
    const std::string link = game::itemChatLink(att->itemId, static_cast<uint32_t>(info->quality), info->name);
    lua_pushstring(L, link.c_str());
    return 1;
}

// TakeInboxItem(index, slot) - an attachment is asked for by its own id, not by
// where it sits in the list
static int lua_TakeInboxItem(lua_State* L) {
    auto* gh = getGameHandler(L);
    const auto* mail = mailAt(gh, static_cast<int>(luaL_optnumber(L, 1, 0)));
    const auto* att = attachmentAt(mail, static_cast<int>(luaL_optnumber(L, 2, 1)));
    if (gh && mail && att) gh->mailTakeItem(mail->messageId, att->itemGuidLow);
    return 0;
}

static int lua_TakeInboxMoney(lua_State* L) {
    auto* gh = getGameHandler(L);
    const auto* mail = mailAt(gh, static_cast<int>(luaL_optnumber(L, 1, 0)));
    if (gh && mail) gh->mailTakeMoney(mail->messageId);
    return 0;
}

static int lua_DeleteInboxItem(lua_State* L) {
    auto* gh = getGameHandler(L);
    const auto* mail = mailAt(gh, static_cast<int>(luaL_optnumber(L, 1, 0)));
    if (gh && mail) gh->mailDelete(mail->messageId);
    return 0;
}

// InboxItemCanDelete(index) - nothing here refuses a deletion
static int lua_InboxItemCanDelete(lua_State* L) {
    lua_pushboolean(L, 1);
    return 1;
}

// AutoLootMailItem(index) - the coin first, then every attachment
static int lua_AutoLootMailItem(lua_State* L) {
    auto* gh = getGameHandler(L);
    const auto* mail = mailAt(gh, static_cast<int>(luaL_optnumber(L, 1, 0)));
    if (!gh || !mail) return 0;
    if (mail->money > 0) gh->mailTakeMoney(mail->messageId);
    for (const auto& att : mail->attachments) {
        gh->mailTakeItem(mail->messageId, att.itemGuidLow);
    }
    return 0;
}

// GetSendMailItem(slot) → name, texture, stackCount, quality
//
// What is attached to the letter being written, and an empty slot is most of
// them: the compose frame walks all twelve every time it updates.
//
// An empty one answers nil, nil, 0, nil - a count of zero rather than no
// count. SendMailFrame_Update writes `if ( stackCount <= 1 )` with nothing in
// front of it, so a nil there is not an empty slot but a comparison against
// nothing, which raises on the first slot and takes the rest of the frame with
// it. Opening the Send Mail tab did exactly that, so the compose frame could
// never be built.
//
// The other three stay nil, which is what the frame wants: the name is tested
// before use, and a nil texture is how a region is told it has nothing to
// draw.
static int lua_GetSendMailItem(lua_State* L) {
    auto* gh = getGameHandler(L);
    // Every early exit answers the empty slot rather than answering nothing,
    // for the reason above. Only the shape of the answer is different from a
    // filled slot, never the number of values.
    auto emptySlot = [](lua_State* s) {
        lua_pushnil(s);            // name
        lua_pushnil(s);            // texture
        lua_pushnumber(s, 0);      // stackCount, and it is compared unguarded
        lua_pushnil(s);            // quality
        return 4;
    };
    const int slot = static_cast<int>(luaL_optnumber(L, 1, 0));
    if (!gh || slot < 1) { return emptySlot(L); }
    const auto& attachments = gh->getMailAttachments();
    if (slot > static_cast<int>(attachments.size())) { return emptySlot(L); }
    const auto& att = attachments[slot - 1];
    if (!att.occupied()) { return emptySlot(L); }

    const auto* info = gh->getItemInfo(att.item.itemId);
    lua_pushstring(L, info ? info->name.c_str() : "");
    lua_pushstring(L, gh->getItemIconPath(
        info && info->displayInfoId ? info->displayInfoId : att.item.displayInfoId).c_str());
    lua_pushnumber(L, att.item.stackCount);
    lua_pushnumber(L, info ? info->quality : 1);
    return 4;
}

// ReturnInboxItem(index) - send a letter back where it came from, with
// whatever is still attached to it.
//
// The one inbox action that was missing: taking the money, taking an
// attachment and deleting were all here, and returning was not, so a letter
// that should have gone back could only be deleted - which destroys whatever
// came with it.
// GetCoinIcon(copper) → the coin to draw for an amount.
//
// The letter's money attachment is drawn with SetItemButtonTexture, and a nil
// texture reads as an empty slot to FrameXML - so a letter carrying gold showed
// nothing attached at all. Gold above a gold, silver above a silver, copper
// below: the three icon paths are the ones globalstrings names in
// GOLD_AMOUNT_TEXTURE and its pair, so this is the interface's own artwork
// rather than a path invented to fill the gap.
static int lua_GetCoinIcon(lua_State* L) {
    constexpr double kCopperPerSilver = 100.0;
    constexpr double kCopperPerGold   = 100.0 * 100.0;
    const double copper = luaL_optnumber(L, 1, 0);
    const char* icon = (copper >= kCopperPerGold)   ? "Interface\\MoneyFrame\\UI-GoldIcon"
                     : (copper >= kCopperPerSilver) ? "Interface\\MoneyFrame\\UI-SilverIcon"
                                                    : "Interface\\MoneyFrame\\UI-CopperIcon";
    lua_pushstring(L, icon);
    return 1;
}

static int lua_ReturnInboxItem(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int index = static_cast<int>(luaL_optnumber(L, 1, 0));
    if (!gh || index < 1) return 0;
    const auto& mail = gh->getMailInbox();
    if (index > static_cast<int>(mail.size())) return 0;
    gh->mailReturnToSender(mail[static_cast<size_t>(index - 1)].messageId);
    return 0;
}

static int lua_CheckInbox(lua_State* L) {
    if (auto* gh = getGameHandler(L)) gh->refreshMailList();
    return 0;
}

// --- Money attached to the letter being written ---
//
// SendMail carries an amount and a cash-on-delivery charge, and the compose
// frame sets them before it sends: it reads the copper out of its own money
// input, then calls SetSendMailMoney or SetSendMailCOD depending on which of
// the two buttons is checked. Neither of those existed, so the amount the
// player typed reached nothing and every letter was sent with SendMail's money
// and COD arguments hard-coded to zero.
//
// That failure is quiet in the worst way. Nothing raises and nothing is logged;
// the letter simply arrives empty, and the sender has no reason to think it did
// not work until whoever received it says so.
//
// The amount belongs here rather than on the game handler because it is not
// game state - the server is told it once, as an argument to the send. It is
// cleared after a send and when the mailbox closes so that a figure typed and
// then abandoned cannot attach itself to the next letter.
namespace {
uint32_t s_sendMailMoney = 0;
uint32_t s_sendMailCOD   = 0;

/// Copper from Lua, refusing negatives rather than wrapping them into a huge
/// unsigned amount.
uint32_t copperArg(lua_State* L, int index) {
    const double v = luaL_optnumber(L, index, 0);
    if (!(v > 0)) return 0;  // also catches NaN
    return static_cast<uint32_t>(v);
}
} // namespace

static int lua_CloseMail(lua_State* L) {
    s_sendMailMoney = 0;
    s_sendMailCOD = 0;
    if (auto* gh = getGameHandler(L)) gh->closeMailbox();
    return 0;
}

static int lua_SetSendMailMoney(lua_State* L) {
    s_sendMailMoney = copperArg(L, 1);
    return 0;
}

static int lua_SetSendMailCOD(lua_State* L) {
    s_sendMailCOD = copperArg(L, 1);
    return 0;
}

// The coin pickup frame adds to what is already attached rather than replacing
// it, so these are not the setters under another name.
// Called bare by the money frame's DropFunc, so an absent argument means the
// cursor's whole amount rather than nothing - which is what `+= copperArg` came
// to, leaving dragged gold out of the letter without a word.
static int lua_AddSendMailMoney(lua_State* L) {
    const uint32_t named = copperArg(L, 1);
    if (named > 0) { s_sendMailMoney += named; return 0; }
    const uint64_t held = cursorMoney();
    if (held == 0) return 0;
    s_sendMailMoney += static_cast<uint32_t>(held);
    setCursorMoney(L, 0);
    return 0;
}

/// Take the money back off the letter and onto the cursor.
static int lua_PickupSendMailMoney(lua_State* L) {
    uint64_t take = static_cast<uint64_t>(luaL_optnumber(L, 1, 0));
    if (s_sendMailMoney == 0) return 0;
    if (take == 0 || take > s_sendMailMoney) take = s_sendMailMoney;
    s_sendMailMoney -= static_cast<uint32_t>(take);
    setCursorMoney(L, take);
    return 0;
}

static int lua_AddSendMailCOD(lua_State* L) {
    s_sendMailCOD += copperArg(L, 1);
    return 0;
}

static int lua_GetSendMailMoney(lua_State* L) {
    lua_pushnumber(L, static_cast<double>(s_sendMailMoney));
    return 1;
}

static int lua_GetSendMailCOD(lua_State* L) {
    lua_pushnumber(L, static_cast<double>(s_sendMailCOD));
    return 1;
}

// SendMail(recipient, subject, body) - the money and the cash-on-delivery were
// set separately by the interface before this was called.
static int lua_SendMail(lua_State* L) {
    auto* gh = getGameHandler(L);
    const char* to = luaL_optstring(L, 1, "");
    const char* subject = luaL_optstring(L, 2, "");
    const char* body = luaL_optstring(L, 3, "");
    if (gh && to && *to) gh->sendMail(to, subject, body, s_sendMailMoney, s_sendMailCOD);
    // Whether or not it went, the next letter starts empty. Leaving the amount
    // set would attach it again to a letter nobody meant to put money in.
    s_sendMailMoney = 0;
    s_sendMailCOD = 0;
    return 0;
}

// --- Equipment sets ---
//
// These live on the server: it sends the list, and saving, using and deleting
// are requests. This client already receives and keeps all of it - the names,
// the icons, the item in each slot and the slots a set was told to ignore - so
// these read that rather than keeping a second set of sets on this side. A
// local copy would not be the character's sets, and anything saved through it
// would not exist for anyone else.
namespace {

constexpr int kEquipSlots = static_cast<int>(game::EquipSlot::BAG1);

const game::EquipmentSetInfo* equipmentSetByName(game::GameHandler* gh,
                                                 const std::string& name) {
    if (!gh) return nullptr;
    for (const auto& set : gh->getEquipmentSets()) {
        if (set.name == name) return &set;
    }
    return nullptr;
}

/// How much of a set is worn, and how much is merely to hand.
struct SetTally { int items = 0, equipped = 0, inBags = 0, missing = 0, ignored = 0; };

SetTally tallySet(game::GameHandler* gh, const game::EquipmentSetInfo& set) {
    SetTally t;
    const auto* guids = gh ? gh->getEquipmentSetItems(set.setId) : nullptr;
    if (!gh || !guids) return t;
    const uint32_t ignoreMask = gh->getEquipmentSetIgnoreMask(set.setId);
    const auto& inv = gh->getInventory();

    for (int i = 0; i < kEquipSlots; ++i) {
        if (ignoreMask & (1u << i)) { ++t.ignored; continue; }
        const uint32_t want = gh->getItemIdByGuid((*guids)[i]);
        if (want == 0) { ++t.ignored; continue; }
        ++t.items;
        // Worn anywhere counts as worn: a ring saved from one ring slot and put
        // back on in the other is on the player either way.
        bool onBody = false;
        for (int w = 0; w < kEquipSlots && !onBody; ++w) {
            const auto& worn = inv.getEquipSlot(static_cast<game::EquipSlot>(w));
            onBody = !worn.empty() && worn.item.itemId == want;
        }
        if (onBody) { ++t.equipped; continue; }

        bool found = false;
        for (int b = 0; b < inv.getBackpackSize() && !found; ++b) {
            const auto& sl = inv.getBackpackSlot(b);
            found = !sl.empty() && sl.item.itemId == want;
        }
        for (int bag = 0; bag < game::Inventory::NUM_BAG_SLOTS && !found; ++bag) {
            for (int sl = 0; sl < inv.getBagSize(bag) && !found; ++sl) {
                const auto& s2 = inv.getBagSlot(bag, sl);
                found = !s2.empty() && s2.item.itemId == want;
            }
        }
        if (found) ++t.inBags; else ++t.missing;
    }
    return t;
}

/// Slots the player marked to leave alone before the next save. A choice about
/// the save to come rather than part of any set, so it is kept here.
std::set<int>& pendingIgnoredSlots() {
    static std::set<int> slots;
    return slots;
}

}  // namespace

static int lua_GetNumEquipmentSets(lua_State* L) {
    auto* gh = getGameHandler(L);
    lua_pushnumber(L, gh ? static_cast<double>(gh->getEquipmentSets().size()) : 0.0);
    return 1;
}

static void pushSetInfo(lua_State* L, game::GameHandler* gh,
                        const game::EquipmentSetInfo& set) {
    const SetTally t = tallySet(gh, set);
    lua_pushstring(L, set.name.c_str());
    // The server names the icon; the interface wants something SetTexture can
    // use, and every one of them lives under the same directory.
    if (set.iconName.empty()) lua_pushnil(L);
    else lua_pushstring(L, ("Interface\\Icons\\" + set.iconName).c_str());
    lua_pushnumber(L, set.setId);
    lua_pushboolean(L, (t.items > 0 && t.equipped == t.items) ? 1 : 0);
    lua_pushnumber(L, t.items);
    lua_pushnumber(L, t.equipped);
    lua_pushnumber(L, t.inBags);
    lua_pushnumber(L, t.missing);
    lua_pushnumber(L, t.ignored);
}

static int lua_GetEquipmentSetInfo(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int index = static_cast<int>(luaL_optnumber(L, 1, 0));
    if (!gh) { return luaReturnNil(L); }
    const auto& sets = gh->getEquipmentSets();
    if (index < 1 || index > static_cast<int>(sets.size())) { return luaReturnNil(L); }
    pushSetInfo(L, gh, sets[index - 1]);
    return 9;
}

static int lua_GetEquipmentSetInfoByName(lua_State* L) {
    auto* gh = getGameHandler(L);
    const auto* set = equipmentSetByName(gh, luaL_optstring(L, 1, ""));
    if (!set) { return luaReturnNil(L); }
    pushSetInfo(L, gh, *set);
    return 9;
}

// GetEquipmentSetItemIDs(name) → one entry per slot, walked with ipairs
static int lua_GetEquipmentSetItemIDs(lua_State* L) {
    auto* gh = getGameHandler(L);
    const auto* set = equipmentSetByName(gh, luaL_optstring(L, 1, ""));
    const auto* guids = (gh && set) ? gh->getEquipmentSetItems(set->setId) : nullptr;
    const uint32_t ignoreMask = (gh && set) ? gh->getEquipmentSetIgnoreMask(set->setId) : 0u;
    lua_newtable(L);
    for (int i = 0; i < kEquipSlots; ++i) {
        // Dense from one: the interface reads this with ipairs and would stop
        // at the first hole. An ignored slot is one, which is what the
        // interface reads as EQUIPMENT_SET_IGNORED_SLOT.
        lua_pushnumber(L, i + 1);
        if (ignoreMask & (1u << i)) {
            lua_pushnumber(L, 1);
        } else {
            lua_pushnumber(L, guids ? gh->getItemIdByGuid((*guids)[i]) : 0);
        }
        lua_settable(L, -3);
    }
    return 1;
}

// SaveEquipmentSet(name, iconIndex) - asks the server to keep it
static int lua_SaveEquipmentSet(lua_State* L) {
    auto* gh = getGameHandler(L);
    const std::string name = luaL_optstring(L, 1, "");
    if (!gh || name.empty()) return 0;

    // The icon arrives as a position in the same list the macro picker shows,
    // and the server wants its name rather than its path.
    std::string iconName = "INV_Misc_QuestionMark";
    const int iconIndex = static_cast<int>(luaL_optnumber(L, 2, 0));
    if (auto* svc = getLuaServices(L); svc && svc->listIconTextures && iconIndex >= 1) {
        const auto& icons = svc->listIconTextures();
        if (iconIndex <= static_cast<int>(icons.size())) {
            const std::string& path = icons[static_cast<size_t>(iconIndex - 1)];
            const size_t slash = path.find_last_of("\\/");
            iconName = (slash == std::string::npos) ? path : path.substr(slash + 1);
        }
    }
    // Overwriting one keeps its guid, which is how the server knows it is the
    // same set rather than a new one with the same name.
    const auto* existing = equipmentSetByName(gh, name);
    gh->saveEquipmentSet(name, iconName, existing ? existing->setGuid : 0,
                         existing ? existing->setId : 0xFFFFFFFFu);
    return 0;
}

static int lua_DeleteEquipmentSet(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (const auto* set = equipmentSetByName(gh, luaL_optstring(L, 1, ""))) {
        gh->deleteEquipmentSet(set->setGuid);
    }
    return 0;
}

static int lua_UseEquipmentSet(lua_State* L) {
    auto* gh = getGameHandler(L);
    const auto* set = equipmentSetByName(gh, luaL_optstring(L, 1, ""));
    if (!gh || !set) { lua_pushboolean(L, 0); return 1; }
    gh->useEquipmentSet(set->setId);
    // The manager waits on this before refreshing and clearing the slots it was
    // told to ignore.
    gh->fireAddonEvent("EQUIPMENT_SWAP_FINISHED", {"1", set->name});
    lua_pushboolean(L, 1);
    return 1;
}

// EquipmentSetContainsLockedItems(name) - nothing here locks an item
static int lua_EquipmentSetContainsLockedItems(lua_State* L) {
    lua_pushboolean(L, 0);
    return 1;
}

static int lua_EquipmentManagerIgnoreSlotForSave(lua_State* L) {
    pendingIgnoredSlots().insert(static_cast<int>(luaL_optnumber(L, 1, 0)));
    return 0;
}
static int lua_EquipmentManagerUnignoreSlotForSave(lua_State* L) {
    pendingIgnoredSlots().erase(static_cast<int>(luaL_optnumber(L, 1, 0)));
    return 0;
}
static int lua_EquipmentManagerClearIgnoredSlotsForSave(lua_State* L) {
    (void)L;
    pendingIgnoredSlots().clear();
    return 0;
}

// GetInventoryItemsForSlot(slotId, table) - everything that could go in a slot
//
// Fills the caller's table with location -> itemID, where the location is packed
// the way the equipment manager unpacks it: a flag for where it is, and for a
// bag the bag index shifted left eight with the slot in the low bits. The
// interface subtracts the slot id from the equipped entry to recognise and drop
// it, so the equipped item has to be in there too.
static int lua_GetInventoryItemsForSlot(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int slotId = static_cast<int>(luaL_optnumber(L, 1, 0));
    if (!gh || slotId < 1 || slotId > kEquipSlots || !lua_istable(L, 2)) return 0;

    constexpr uint32_t kLocationPlayer = 0x00100000;
    constexpr uint32_t kLocationBags   = 0x00200000;
    constexpr int      kBagBitOffset   = 8;

    // Which inventory types the server sends fit which slot. A flyout that is a
    // little generous about weapons is better than one that hides a sword,
    // so a one-handed weapon is offered for either hand.
    // Named rather than numbered: this client already spells the inventory
    // types out, and the two agreed when checked against each other.
    namespace IT = game::InvType;
    auto fits = [](int slot, uint32_t t) {
        switch (slot) {
            case 1:  return t == IT::HEAD;
            case 2:  return t == IT::NECK;
            case 3:  return t == IT::SHOULDERS;
            case 4:  return t == IT::SHIRT;
            case 5:  return t == IT::CHEST || t == IT::ROBE;
            case 6:  return t == IT::WAIST;
            case 7:  return t == IT::LEGS;
            case 8:  return t == IT::FEET;
            case 9:  return t == IT::WRISTS;
            case 10: return t == IT::HANDS;
            case 11: case 12: return t == IT::FINGER;     // the two rings
            case 13: case 14: return t == IT::TRINKET;    // the two trinkets
            case 15: return t == IT::BACK;
            case 16: return t == IT::ONE_HAND || t == IT::TWO_HAND || t == IT::MAIN_HAND;
            case 17: return t == IT::ONE_HAND || t == IT::SHIELD ||
                            t == IT::OFF_HAND || t == IT::HOLDABLE;
            // Relics go in the ranged slot and have no name here, the list
            // stopping at guns; twenty-eight is what the server sends for one.
            case 18: return t == IT::RANGED_BOW || t == IT::THROWN ||
                            t == IT::RANGED_GUN || t == 28;
            case 19: return t == IT::TABARD;
            default: return false;
        }
    };
    auto offer = [&](uint32_t location, uint32_t itemId) {
        lua_pushnumber(L, static_cast<double>(location));
        lua_pushnumber(L, static_cast<double>(itemId));
        lua_settable(L, 2);
    };
    auto invTypeOf = [&](uint32_t itemId) -> uint32_t {
        const auto* info = gh->getItemInfo(itemId);
        return info ? info->inventoryType : 0u;
    };

    const auto& inv = gh->getInventory();
    const auto& worn = inv.getEquipSlot(static_cast<game::EquipSlot>(slotId - 1));
    if (!worn.empty()) {
        offer(kLocationPlayer + static_cast<uint32_t>(slotId), worn.item.itemId);
    }
    // Bag zero is the backpack, and slots count from one, which is how the
    // interface reads them back out with GetContainerItemInfo.
    for (int i = 0; i < inv.getBackpackSize(); ++i) {
        const auto& sl = inv.getBackpackSlot(i);
        if (sl.empty() || !fits(slotId, invTypeOf(sl.item.itemId))) continue;
        offer(kLocationBags | static_cast<uint32_t>(i + 1), sl.item.itemId);
    }
    for (int bag = 0; bag < game::Inventory::NUM_BAG_SLOTS; ++bag) {
        for (int i = 0; i < inv.getBagSize(bag); ++i) {
            const auto& sl = inv.getBagSlot(bag, i);
            if (sl.empty() || !fits(slotId, invTypeOf(sl.item.itemId))) continue;
            const uint32_t location = kLocationBags |
                (static_cast<uint32_t>(bag + 1) << kBagBitOffset) |
                static_cast<uint32_t>(i + 1);
            offer(location, sl.item.itemId);
        }
    }
    return 0;
}


// --- Items named rather than pointed at ---
//
// /use and /equip take what the player typed, so these accept a name or a link
// and look through what is carried. A link is preferred when given, because two
// items can share a name and only the link says which.

/// The first carried item whose id or name matches, or zero.
static uint32_t carriedItemMatching(game::GameHandler* gh, lua_State* L, int arg) {
    if (!gh) return 0;
    const uint32_t byId = itemIdFromArg(L, arg);
    std::string byName = lua_isstring(L, arg) ? lua_tostring(L, arg) : "";
    // A link parsed to an id above; treat the text as a name only otherwise.
    if (byId != 0) byName.clear();

    const auto& inv = gh->getInventory();
    auto matches = [&](uint32_t itemId) {
        if (itemId == 0) return false;
        if (byId != 0) return itemId == byId;
        if (byName.empty()) return false;
        const auto* info = gh->getItemInfo(itemId);
        return info && info->name == byName;
    };
    for (int i = 0; i < inv.getBackpackSize(); ++i) {
        const auto& sl = inv.getBackpackSlot(i);
        if (!sl.empty() && matches(sl.item.itemId)) return sl.item.itemId;
    }
    for (int bag = 0; bag < game::Inventory::NUM_BAG_SLOTS; ++bag) {
        for (int i = 0; i < inv.getBagSize(bag); ++i) {
            const auto& sl = inv.getBagSlot(bag, i);
            if (!sl.empty() && matches(sl.item.itemId)) return sl.item.itemId;
        }
    }
    return 0;
}

// GetInventoryItemDurability(slot) → current, maximum
//
// Absent for an item that cannot be damaged, which is what the durability
// frame reads to decide whether the slot is worth drawing at all.
static int lua_GetInventoryItemDurability(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int slotId = static_cast<int>(luaL_optnumber(L, 1, 0));
    if (!gh || slotId < 1 || slotId > kEquipSlots) { return luaReturnNil(L); }
    const auto& sl = gh->getInventory().getEquipSlot(
        static_cast<game::EquipSlot>(slotId - 1));
    if (sl.empty() || sl.item.maxDurability == 0) { return luaReturnNil(L); }
    lua_pushnumber(L, sl.item.curDurability);
    lua_pushnumber(L, sl.item.maxDurability);
    return 2;
}

// UseItemByName(item) - what /use does
/// The unit a /use was aimed at, or zero for "as the item would go anyway".
///
/// chatframe passes one to all three of the use bindings -
/// UseContainerItem(bag, slot, target), UseInventoryItem(slot, target) and
/// UseItemByName(name, target) - and none of them read it, so
/// `/use [target=Bob] <bandage>` bandaged whoever asked. An unresolvable name
/// answers zero rather than refusing: the item then goes where it always did,
/// which is better than a click that does nothing.
uint64_t usedOnUnit(lua_State* L, int index) {
    auto* gh = getGameHandler(L);
    const char* unit = gh ? lua_tostring(L, index) : nullptr;
    if (!unit || !*unit) return 0;
    std::string uid(unit);
    toLowerInPlace(uid);
    return resolveUnitGuid(gh, uid);
}

static int lua_UseItemByName(lua_State* L) {
    auto* gh = getGameHandler(L);
    const uint32_t itemId = carriedItemMatching(gh, L, 1);
    if (gh && itemId != 0) gh->useItemById(itemId, usedOnUnit(L, 2));
    return 0;
}

// EquipItemByName(item) - what /equip does
/// EquipItemByName(item [, slot]) - wear it, optionally in a named slot.
///
/// The second argument was read by nothing, so `/equipslot 12 Some Trinket`
/// picked whichever trinket slot the server preferred and the number the player
/// typed did nothing. It is the interface's one-based inventory slot; the wire
/// counts equipment slots from zero, which is also what this client's EquipSlot
/// enum does.
static int lua_EquipItemByName(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) return 0;
    const uint32_t itemId = carriedItemMatching(gh, L, 1);
    if (itemId == 0) return 0;
    const int wantSlot = static_cast<int>(luaL_optnumber(L, 2, 0));
    // Zero is "no slot named", not slot zero - the interface's own numbering
    // starts at one, so there is no ambiguity to resolve.
    if (wantSlot >= 1 && wantSlot <= game::Inventory::NUM_EQUIP_SLOTS) {
        const uint64_t guid = gh->resolveOnlineItemGuid(itemId);
        if (guid != 0) {
            gh->equipItemToSlot(guid, static_cast<uint8_t>(wantSlot - 1));
            return 0;
        }
        // No guid for it - fall through and let the server choose, which is
        // better than doing nothing at all.
    }
    const auto& inv = gh->getInventory();
    for (int i = 0; i < inv.getBackpackSize(); ++i) {
        const auto& sl = inv.getBackpackSlot(i);
        if (!sl.empty() && sl.item.itemId == itemId) { gh->autoEquipItemBySlot(i); return 0; }
    }
    for (int bag = 0; bag < game::Inventory::NUM_BAG_SLOTS; ++bag) {
        for (int i = 0; i < inv.getBagSize(bag); ++i) {
            const auto& sl = inv.getBagSlot(bag, i);
            if (!sl.empty() && sl.item.itemId == itemId) {
                gh->autoEquipItemInBag(bag, i);
                return 0;
            }
        }
    }
    return 0;
}

// IsEquippableItem(item) - whether it has a slot to go in at all
static int lua_IsEquippableItem(lua_State* L) {
    auto* gh = getGameHandler(L);
    const uint32_t itemId = itemIdFromArg(L, 1);
    const auto* info = (gh && itemId) ? gh->getItemInfo(itemId) : nullptr;
    // Zero is "nowhere to wear it" - reagents, food, quest items.
    lua_pushboolean(L, (info && info->inventoryType != 0) ? 1 : 0);
    return 1;
}

// IsEquippedItem(item) - whether it is being worn right now
static int lua_IsEquippedItem(lua_State* L) {
    auto* gh = getGameHandler(L);
    const uint32_t itemId = itemIdFromArg(L, 1);
    bool worn = false;
    if (gh && itemId != 0) {
        const auto& inv = gh->getInventory();
        for (int i = 0; i < kEquipSlots && !worn; ++i) {
            const auto& sl = inv.getEquipSlot(static_cast<game::EquipSlot>(i));
            worn = !sl.empty() && sl.item.itemId == itemId;
        }
    }
    lua_pushboolean(L, worn ? 1 : 0);
    return 1;
}

// UseInventoryItem(slot) - use what is equipped in a slot
//
// How a trinket is clicked on the character sheet. The slot numbers are the
// interface's, one-based, and the item is used by id the same way the client's
// own paperdoll uses it.
static int lua_UseInventoryItem(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int slotId = static_cast<int>(luaL_optnumber(L, 1, 0));
    if (!gh || slotId < 1 || slotId > 19) return 0;
    // A weapon that is worn is as good a target as one in a bag, and the
    // paperdoll's right-click arrives here.
    if (completedItemTarget(L, gh->getEquipSlotGuid(slotId - 1))) return 0;
    const auto& slot = *inventorySlotItem(gh->getInventory(), slotId);
    if (!slot.empty()) gh->useItemById(slot.item.itemId, usedOnUnit(L, 2));
    return 0;
}

// CloseBankFrame() - tell the server the bank is done with
static int lua_CloseBankFrame(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (gh) gh->closeBank();
    return 0;
}

// UseContainerItem(bag, slot) - use/equip an item from a bag
static int lua_UseContainerItem(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) return 0;
    int bag = static_cast<int>(luaL_checknumber(L, 1));
    int slot = static_cast<int>(luaL_checknumber(L, 2));
    // The click that picks what a stone or an oil is applied to. FrameXML
    // routes it here rather than to the pickup when SpellCanTargetItem says a
    // spell is waiting for one.
    if (completedItemTarget(L, containerSlotGuid(gh, bag, slot))) return 0;
    const auto& inv = gh->getInventory();
    // The keyring counts here too: right-clicking a key is how a locked box or
    // a quest door is opened, and the branch this replaced could not name it.
    const game::ItemSlot* itemSlot = containerItemSlot(inv, bag, slot);
    if (!itemSlot || itemSlot->empty()) return 0;

    // Right-clicking an item in a bag is three different messages depending on
    // what the item is, and only the last of them was sent from here. Taken
    // from this client's own bag window, which is where the distinction was
    // already drawn and which is handed over.
    //
    // Openable is an item flag rather than an item-class guarantee - some
    // fishing containers are miscellaneous items - so both the flag and the
    // container class count, or those fall through to a CMSG_USE_ITEM the
    // server silently ignores.
    constexpr uint32_t kItemFlagOpenable = 0x00000004u;
    const auto* info = gh->getItemInfo(itemSlot->item.itemId);
    const bool readable = info && info->valid && info->pageTextId != 0;
    const bool openable = info && info->valid &&
                          ((info->itemFlags & kItemFlagOpenable) != 0 ||
                           info->itemClass == 1);

    // What an open window makes a right-click mean, in the order this client's
    // own bag window decided it - mail, bank, guild bank, merchant - because
    // that window is the specification and it is the one being suppressed. The
    // real client reads UseContainerItem the same way and containerframe.lua
    // has no branch for any of them: line 733 routes every right-click here and
    // the C function decides.
    //
    // Each of these verbs had exactly one caller, in the window handed over, so
    // handing the bags to FrameXML left no way to attach mail, bank an item or
    // stock a guild bank at all - and a right-click ran on to the branches
    // below and *used* the item instead.
    //
    // Trade is deliberately not here. setTradeItem needs a slot to put the item
    // in and this client's own window has never offered a right-click for it,
    // so a branch would be inventing behaviour rather than restoring it; a
    // trade is made by dragging, which the cursor bridge carries.
    // A key, before any of the window branches below. Those read the container
    // as a worn bag - attachItemFromBag(bag - 1, ...) and sellItemInBag do -
    // and a key cannot be mailed, sold or banked in any case: the keyring is
    // the only place one can live, so using it is the only thing a right-click
    // can mean.
    if (bag == kKeyringContainer) {
        gh->useKeyringItem(slot - 1);
        return 0;
    }

    const int wireSlot = containerWireSlot(bag, slot);
    const uint8_t wireBag = containerWireBag(bag);

    if (gh->isMailComposeOpen()) {
        if (bag == 0) gh->attachItemFromBackpack(slot - 1);
        else          gh->attachItemFromBag(bag - 1, slot - 1);
        return 0;
    }

    // depositItem sends CMSG_AUTOBANK_ITEM, so the server finds the first free
    // slot across the main bank and every purchased bank bag rather than this
    // choosing one.
    if (gh->isBankOpen()) {
        // Which way depends on which side the item is already on. A
        // right-click in a bank bag takes it out; anywhere else puts it in.
        const bool fromBank = isBankBagContainer(bag) || bag == kBankContainer;
        if (fromBank) gh->withdrawItem(wireBag, static_cast<uint8_t>(wireSlot));
        else          gh->depositItem(wireBag, static_cast<uint8_t>(wireSlot));
        return 0;
    }

    // A bank bag with the bank shut cannot be reached - CloseBankBagFrames
    // shuts all seven on BANKFRAME_CLOSED - and every branch past here reads
    // the container as a worn bag. Refusing is the honest answer to a state
    // that should not arise, and it is what kept the keyring out of them.
    if (isBankBagContainer(bag) || bag == kBankContainer) return 0;

    if (gh->isGuildBankOpen()) {
        gh->guildBankDepositFromInventory(wireBag, static_cast<uint8_t>(wireSlot));
        return 0;
    }

    // SellContainerItem is this client's own invention and FrameXML has never
    // heard of it, so the sell verbs sat behind a name nothing calls.
    // containerframe.lua has already returned early for the buyback tab and for
    // anything needing a price confirmation before it reaches here.
    if (gh->isVendorWindowOpen()) {
        if (bag == 0) gh->sellItemBySlot(slot - 1);
        else          gh->sellItemInBag(bag - 1, slot - 1);
        return 0;
    }

    // Something with an equip slot is equipped by a right-click, not used.
    // That is a different message - CMSG_AUTOEQUIP_ITEM rather than
    // CMSG_USE_ITEM - and this only ever sent the second, so right-clicking
    // gear in a bag did nothing at all. It holds for a trinket too: one in a
    // bag is equipped, and only one already worn is used.
    //
    // A lockbox and its kind have no equip slot, so they still open; a bag has
    // one, so it goes into a bag slot the way it should.
    const bool equippable = info && info->valid && info->inventoryType != 0;

    // An item that begins a quest offers it, and that is not a use at all.
    // AzerothCore starts an item quest from CMSG_QUESTGIVER_QUERY_QUEST naming
    // the *item's* guid - HandleQuestgiverQueryQuestOpcode accepts TYPEMASK_ITEM
    // - and does nothing with CMSG_USE_ITEM for one. This client's own bag
    // window checked it first for that reason and was the only caller, so with
    // the bags handed over a quest item answered a right-click with nothing at
    // all.
    if (itemSlot->item.startQuestId != 0) {
        const uint64_t itemGuid = containerSlotGuid(gh, bag, slot);
        if (itemGuid != 0) {
            gh->offerQuestFromItem(itemGuid, itemSlot->item.startQuestId);
            return 0;
        }
    }

    // By slot rather than by item id: the id searches the bags for a match and
    // can find a different stack of the same thing than the one clicked.
    if (bag == 0) {
        const int idx = slot - 1;
        if (readable)         gh->readItemBySlot(idx);
        else if (equippable)  gh->autoEquipItemBySlot(idx);
        else if (openable)    gh->openItemBySlot(idx);
        else                  gh->useItemBySlot(idx, false, usedOnUnit(L, 3));
    } else {
        if (readable)         gh->readItemInBag(bag - 1, slot - 1);
        else if (equippable)  gh->autoEquipItemInBag(bag - 1, slot - 1);
        else if (openable)    gh->openItemInBag(bag - 1, slot - 1);
        else                  gh->useItemInBag(bag - 1, slot - 1, false,
                                             usedOnUnit(L, 3));
    }
    return 0;
}

// ── The auction panel's column sort ────────────────────────────────────────
//
// The server sends a list; the client sorts it. There is no re-query, which is
// why all four of these used to be no-ops and nothing looked obviously broken:
// the rows were there, just always in the order they arrived.
//
// What was missing was visible, though. AuctionFrame_OnClickSortColumn reads
// GetAuctionSort to decide whether a second click on the same header should
// reverse, and SortButton_UpdateArrow reads it to decide which header shows an
// arrow and which way it points. With nothing to read, no header ever showed
// an arrow and every click sorted the same way.
//
// FrameXML builds a sort as a *sequence*: SortAuctionClearSort, then one
// SortAuctionSetSort per column from least significant to most, then
// SortAuctionApplySort. GetAuctionSort(table, 1) asks for the primary, which
// is therefore the one set last.

namespace {

struct AuctionSortKey { std::string column; bool reverse = false; };

/// Per list - "list", "owner", "bidder" - because each tab sorts on its own.
std::unordered_map<std::string, std::vector<AuctionSortKey>>& auctionSortState() {
    static std::unordered_map<std::string, std::vector<AuctionSortKey>> s;
    return s;
}

/// The browse tab's ordering, as the wire wants it.
///
/// The columns FrameXML names and the ones the server sorts on are the same
/// set under different names, so this is the whole of the translation. Any
/// column with no counterpart is dropped rather than guessed at: the server
/// discards a search whose sort block it cannot read, so an invented id costs
/// the entire result rather than one column of ordering.
///
/// Order is reversed on the way out. FrameXML pushes keys least significant
/// first - GetAuctionSort(table, 1) is the one set *last* - and the server
/// walks its vector from the front, taking the first as primary.
std::vector<game::AuctionSortKey> wireAuctionSort(std::string_view which) {
    static const std::unordered_map<std::string, uint8_t> kColumns = {
        {"level",        0},   // AUCTION_SORT_MINLEVEL
        {"quality",      1},   // AUCTION_SORT_RARITY
        {"buyout",       2},   // AUCTION_SORT_BUYOUT
        {"duration",     3},   // AUCTION_SORT_TIMELEFT
        {"name",         5},   // AUCTION_SORT_ITEM
        {"minbidbuyout", 6},   // AUCTION_SORT_MINBIDBUY
        {"seller",       7},   // AUCTION_SORT_OWNER
        {"bid",          8},   // AUCTION_SORT_BID
        {"quantity",     9},   // AUCTION_SORT_STACK
    };
    const auto& keys = auctionSortState()[std::string(which)];
    std::vector<game::AuctionSortKey> out;
    out.reserve(keys.size());
    for (const auto& key : std::views::reverse(keys)) {
        auto found = kColumns.find(key.column);
        if (found == kColumns.end()) continue;
        out.push_back({found->second, key.reverse});
    }
    return out;
}

game::AuctionListResult* auctionListForSort(game::GameHandler* gh,
                                            std::string_view which) {
    if (!gh) return nullptr;
    if (which == "owner")  return &gh->auctionOwnerResultsRef();
    if (which == "bidder") return &gh->auctionBidderResultsRef();
    return &gh->auctionBrowseResultsRef();
}

/// Orders two rows on one column.
///
/// Every row gets a defined key, including one whose item template has not
/// arrived - it falls back to exactly what GetAuctionItemInfo displays for it
/// ("Item #1234", level 0, quality 1), so the order matches what is on screen.
///
/// That is not tidiness. The first version answered "cannot compare" for a row
/// with no template and let the caller treat the pair as equal, which makes
/// the comparator not a strict weak ordering: an unknown row is equivalent to
/// every known one, while the known ones order among themselves. std::sort and
/// std::stable_sort are undefined on such a comparator - not merely wrong.
bool auctionLess(game::GameHandler* gh, const std::string& column,
                 const game::AuctionEntry& a, const game::AuctionEntry& b) {
    if (column == "quantity") return a.stackCount < b.stackCount;
    if (column == "duration") return a.timeLeftMs < b.timeLeftMs;
    if (column == "bid") {
        // What the row shows: the running bid where there is one, the opening
        // price where there is not.
        const uint32_t av = a.currentBid ? a.currentBid : a.startBid;
        const uint32_t bv = b.currentBid ? b.currentBid : b.startBid;
        return av < bv;
    }
    if (column == "minbidbuyout") return a.buyoutPrice < b.buyoutPrice;
    if (column == "status") return a.bidderGuid < b.bidderGuid;
    if (column == "seller") return a.ownerGuid < b.ownerGuid;

    const auto* ia = gh ? gh->getItemInfo(a.itemEntry) : nullptr;
    const auto* ib = gh ? gh->getItemInfo(b.itemEntry) : nullptr;
    if (column == "name") {
        const std::string na = ia ? ia->name : "Item #" + std::to_string(a.itemEntry);
        const std::string nb = ib ? ib->name : "Item #" + std::to_string(b.itemEntry);
        return na < nb;
    }
    if (column == "level") {
        return (ia ? ia->requiredLevel : 0u) < (ib ? ib->requiredLevel : 0u);
    }
    if (column == "quality") {
        return (ia ? ia->quality : 1u) < (ib ? ib->quality : 1u);
    }
    // A column this does not know orders nothing, which is a valid ordering:
    // every row is equivalent, so a stable sort leaves the list alone.
    return false;
}

}  // namespace

// _GetItemTooltipData(itemId) → table with armor, bind, stats, damage, description
// Returns a Lua table with detailed item info for tooltip building
static int lua_GetItemTooltipData(lua_State* L) {
    auto* gh = getGameHandler(L);
    uint32_t itemId = static_cast<uint32_t>(luaL_checknumber(L, 1));
    if (!gh || itemId == 0) { return luaReturnNil(L); }
    const auto* info = gh->getItemInfo(itemId);
    if (!info) {
        // Same miss, same request - this is the path a tooltip takes for its
        // stats, and it was the one leaving rings reading "Miscellaneous".
        gh->queryItemInfo(itemId, 0);
        return luaReturnNil(L);
    }

    lua_newtable(L);
    // Unique / Heroic flags
    if (info->maxCount == 1) { lua_pushboolean(L, 1); lua_setfield(L, -2, "isUnique"); }
    if (info->itemFlags & 0x8) { lua_pushboolean(L, 1); lua_setfield(L, -2, "isHeroic"); }
    if (info->itemFlags & 0x1000000) { lua_pushboolean(L, 1); lua_setfield(L, -2, "isUniqueEquipped"); }
    // Bind type
    lua_pushnumber(L, info->bindType);
    lua_setfield(L, -2, "bindType");
    // Armor
    lua_pushnumber(L, info->armor);
    lua_setfield(L, -2, "armor");
    // Damage
    lua_pushnumber(L, info->damageMin);
    lua_setfield(L, -2, "damageMin");
    lua_pushnumber(L, info->damageMax);
    lua_setfield(L, -2, "damageMax");
    lua_pushnumber(L, info->delayMs);
    lua_setfield(L, -2, "speed");
    // Primary stats
    if (info->stamina != 0) { lua_pushnumber(L, info->stamina); lua_setfield(L, -2, "stamina"); }
    if (info->strength != 0) { lua_pushnumber(L, info->strength); lua_setfield(L, -2, "strength"); }
    if (info->agility != 0) { lua_pushnumber(L, info->agility); lua_setfield(L, -2, "agility"); }
    if (info->intellect != 0) { lua_pushnumber(L, info->intellect); lua_setfield(L, -2, "intellect"); }
    if (info->spirit != 0) { lua_pushnumber(L, info->spirit); lua_setfield(L, -2, "spirit"); }
    // Description
    if (!info->description.empty()) {
        lua_pushstring(L, info->description.c_str());
        lua_setfield(L, -2, "description");
    }
    // Required level
    lua_pushnumber(L, info->requiredLevel);
    lua_setfield(L, -2, "requiredLevel");
    // Extra stats (hit, crit, haste, AP, SP, etc.) as array of {type, value} pairs
    if (!info->extraStats.empty()) {
        lua_newtable(L);
        for (size_t i = 0; i < info->extraStats.size(); ++i) {
            lua_newtable(L);
            lua_pushnumber(L, info->extraStats[i].statType);
            lua_setfield(L, -2, "type");
            lua_pushnumber(L, info->extraStats[i].statValue);
            lua_setfield(L, -2, "value");
            lua_rawseti(L, -2, static_cast<int>(i) + 1);
        }
        lua_setfield(L, -2, "extraStats");
    }
    // Resistances
    if (info->fireRes != 0) { lua_pushnumber(L, info->fireRes); lua_setfield(L, -2, "fireRes"); }
    if (info->natureRes != 0) { lua_pushnumber(L, info->natureRes); lua_setfield(L, -2, "natureRes"); }
    if (info->frostRes != 0) { lua_pushnumber(L, info->frostRes); lua_setfield(L, -2, "frostRes"); }
    if (info->shadowRes != 0) { lua_pushnumber(L, info->shadowRes); lua_setfield(L, -2, "shadowRes"); }
    if (info->arcaneRes != 0) { lua_pushnumber(L, info->arcaneRes); lua_setfield(L, -2, "arcaneRes"); }
    // Item spell effects (Use: / Equip: / Chance on Hit:)
    {
        lua_newtable(L);
        int spellCount = 0;
        for (int i = 0; i < 5; ++i) {
            if (info->spells[i].spellId == 0) continue;
            ++spellCount;
            lua_newtable(L);
            lua_pushnumber(L, info->spells[i].spellId);
            lua_setfield(L, -2, "spellId");
            lua_pushnumber(L, info->spells[i].spellTrigger);
            lua_setfield(L, -2, "trigger");
            // Get spell name for display
            const std::string& sName = gh->getSpellName(info->spells[i].spellId);
            if (!sName.empty()) { lua_pushstring(L, sName.c_str()); lua_setfield(L, -2, "name"); }
            // Get description
            // Formatted, not raw: an item's spell carries the same $-token
            // template a spell does, and handing it over untouched put
            // "$s1 damage" on the tooltip.
            const std::string sDesc = gh->formatSpellDescription(
                info->spells[i].spellId,
                gh->getSpellDescription(info->spells[i].spellId));
            if (!sDesc.empty()) { lua_pushstring(L, sDesc.c_str()); lua_setfield(L, -2, "description"); }
            lua_rawseti(L, -2, spellCount);
        }
        if (spellCount > 0) lua_setfield(L, -2, "itemSpells");
        else lua_pop(L, 1);
    }
    // Gem sockets (WotLK/TBC)
    int numSockets = 0;
    for (int i = 0; i < 3; ++i) {
        if (info->socketColor[i] != 0) ++numSockets;
    }
    if (numSockets > 0) {
        lua_newtable(L);
        for (int i = 0; i < 3; ++i) {
            if (info->socketColor[i] != 0) {
                lua_newtable(L);
                lua_pushnumber(L, info->socketColor[i]);
                lua_setfield(L, -2, "color");
                lua_rawseti(L, -2, i + 1);
            }
        }
        lua_setfield(L, -2, "sockets");
    }
    // Item set
    if (info->itemSetId != 0) {
        lua_pushnumber(L, info->itemSetId);
        lua_setfield(L, -2, "itemSetId");
    }
    // Quest-starting item
    if (info->startQuestId != 0) {
        lua_pushboolean(L, 1);
        lua_setfield(L, -2, "startsQuest");
    }
    return 1;
}

// --- Locale/Build/Realm info ---


static int lua_GetContainerNumSlots(lua_State* L) {
    auto* gh = getGameHandler(L);
    int container = static_cast<int>(luaL_checknumber(L, 1));
    if (!gh) { return luaReturnZero(L); }
    // Through containerSlotCount, which knows the keyring. Answering zero for
    // KEYRING_CONTAINER made GetKeyRingSize loop zero times, so the keyring
    // frame was generated with no slots and opened empty - while the keys were
    // being tracked out of PLAYER_FIELD_KEYRING_SLOT_1 the whole time.
    lua_pushnumber(L, containerSlotCount(gh->getInventory(), container));
    return 1;
}

// GetKeyRingSize() → how many keys fit.
//
// Answered zero, from the list of counts that are zero because the thing
// counted cannot exist here - which the keyring can. PutKeyInKeyRing walks
// `for i=1, GetKeyRingSize()` looking for a free slot and, finding none,
// tells the player there are no empty keyring slots: dropping a key on the
// keyring button failed every time, with an error saying the ring was full.
//
// The same count GetContainerNumSlots gives for KEYRING_CONTAINER, through the
// same helper rather than beside it. Those two disagreeing is how this came
// about: the container path was taught about the keyring and this one was not.
static int lua_GetKeyRingSize(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { return luaReturnZero(L); }
    lua_pushnumber(L, containerSlotCount(gh->getInventory(), kKeyringContainer));
    return 1;
}

// GetContainerItemInfo(container, slot) → texture, count, locked, quality, readable, lootable, link
// GetLatestThreeSenders() - who the unread mail is from.
//
// The minimap's mail icon builds its tooltip from these: with a name it says
// "You have mail from", with none just "You have mail". This client has had
// the inbox and the sender names all along and the tooltip only ever managed
// the second, shorter sentence.
//
// Newest first, which is the order the tooltip reads them in, and only unread
// mail - a sender whose letter has been opened is not news. Fewer than three
// is normal and the caller tests each one, so the tail is simply absent.
static int lua_GetLatestThreeSenders(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { lua_pushnil(L); lua_pushnil(L); lua_pushnil(L); return 3; }

    const auto& inbox = gh->getMailInbox();
    int pushed = 0;
    for (auto it = inbox.rbegin(); it != inbox.rend() && pushed < 3; ++it) {
        if (it->read || it->senderName.empty()) continue;
        lua_pushstring(L, it->senderName.c_str());
        ++pushed;
    }
    for (; pushed < 3; ++pushed) lua_pushnil(L);
    return 3;
}

/// PickupEquipmentSetByName(name) - drag a gear set onto the cursor.
///
/// Defined and does nothing, deliberately. The cursor here holds items and
/// spells; there is no state for it to hold a set, and inventing one to be
/// dropped on an action bar is a feature rather than a binding.
///
/// Defined all the same because it is reachable: it hangs off OnDragStart of a
/// set button, guarded only by the set having a name, and equipment sets do
/// exist here - GetNumEquipmentSets answers from a real list. Undefined, the
/// first drag of a saved set would raise. The set still equips from its
/// button, which is the operation; only the drag is lost.
static int lua_PickupEquipmentSetByName(lua_State* L) { (void)L; return 0; }

// What this client does not keep per bag slot: durability and gem sockets.
//
// Both are read straight into locals and guarded before use - paperdollframe
// writes `local broken = ( maxDurability and durability == 0 )`, and the gems
// are not even unpacked there - so absent is both safe and true. Answering a
// full bar or three empty sockets would be a claim about an item nobody
// inspected.
//
// They are bindings rather than parser work, which is worth saying because it
// was called the other way once: the wire carries this detail and this client
// parses it only for equipped items, so it looked like a data gap until the
// caller was read.
static int lua_GetContainerItemDurability(lua_State* L) {
    lua_pushnil(L);   // durability
    lua_pushnil(L);   // maxDurability
    return 2;
}
static int lua_ItemGemsNone(lua_State* L) {
    lua_pushnil(L); lua_pushnil(L); lua_pushnil(L);   // gem1, gem2, gem3
    return 3;
}

// GetContainerFreeSlots(container [, table]) → the empty slots in that bag.
//
// Fills a table the caller supplies rather than building one, which is how the
// equipment manager uses it: it wipes a work table, calls this, then walks the
// result. The table is returned as well, so `if (GetContainerFreeSlots(i, t))`
// passes - an empty bag gives an empty table, and walking that does nothing,
// which is the right outcome rather than a branch skipped.
//
// Only the backpack and the four bags. The bank containers in the caller's
// loop are not tracked per slot here, and answering an empty table for them
// says "nothing free" rather than inventing space that is not there.
static int lua_GetContainerFreeSlots(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int container = static_cast<int>(luaL_checknumber(L, 1));
    // Reuse the caller's table when given one; otherwise make the result.
    if (lua_istable(L, 2)) lua_pushvalue(L, 2);
    else lua_newtable(L);
    if (!gh) return 1;

    const auto& inv = gh->getInventory();
    int next = 1;
    auto addIfEmpty = [&](const game::ItemSlot& sl, int slotIndex) {
        if (!sl.empty()) return;
        lua_pushnumber(L, slotIndex);          // WoW slots are 1-based
        lua_rawseti(L, -2, next++);
    };
    const int slots = containerSlotCount(inv, container);
    for (int i = 1; i <= slots; ++i) {
        if (const game::ItemSlot* sl = containerItemSlot(inv, container, i))
            addIfEmpty(*sl, i);
    }
    return 1;
}

// GetContainerItemID(container, slot) → the item's entry id, or nil.
//
// The same walk GetContainerItemInfo does, answering the one number rather
// than the row. Callers use it to identify an item without caring what it
// looks like - the character sheet asks it of every bag slot when looking for
// something to equip - so it is the cheapest of the container queries and was
// the only one of them missing.
static int lua_GetContainerItemID(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int container = static_cast<int>(luaL_checknumber(L, 1));
    const int slot = static_cast<int>(luaL_checknumber(L, 2));
    if (!gh) { return luaReturnNil(L); }

    const auto& inv = gh->getInventory();
    // One lookup for every container the interface can name, the keyring
    // included: this branched on 0 and 1-4 and let KEYRING_CONTAINER fall
    // through, so the keyring frame opened with nothing in it.
    const game::ItemSlot* itemSlot = containerItemSlot(inv, container, slot);
    // An empty slot has no id, and nil is how that is said - zero would be an
    // item, and the callers test the result rather than compare it.
    if (!itemSlot || itemSlot->empty()) { return luaReturnNil(L); }
    lua_pushnumber(L, itemSlot->item.itemId);
    return 1;
}

static int lua_GetContainerItemInfo(lua_State* L) {
    auto* gh = getGameHandler(L);
    int container = static_cast<int>(luaL_checknumber(L, 1));
    int slot = static_cast<int>(luaL_checknumber(L, 2));
    if (!gh) { return luaReturnNil(L); }

    const auto& inv = gh->getInventory();
    // One lookup for every container the interface can name, the keyring
    // included: this branched on 0 and 1-4 and let KEYRING_CONTAINER fall
    // through, so the keyring frame opened with nothing in it.
    const game::ItemSlot* itemSlot = containerItemSlot(inv, container, slot);

    if (!itemSlot || itemSlot->empty()) { return luaReturnNil(L); }

    // Get item info for quality/icon
    const auto* info = gh->getItemInfo(itemSlot->item.itemId);

    // Texture. Returning nil here is what made FrameXML's bag look empty while
    // it held items: ContainerFrame_Update passes this straight to
    // SetItemButtonTexture, so every occupied slot drew no icon and read as a
    // free one. The resolver has been on GameHandler all along - it is what
    // this client's own bag draws from.
    // The slot carries a display id from the update fields; where it does not,
    // the item's own record has one, which is the source the vendor and loot
    // bindings beside this one use.
    uint32_t displayId = itemSlot->item.displayInfoId;
    if (displayId == 0 && info) displayId = info->displayInfoId;
    const std::string icon = displayId ? gh->getItemIconPath(displayId) : std::string();
    lua_pushstring(L, icon.empty() ? "Interface\\Icons\\INV_Misc_QuestionMark"
                                   : icon.c_str());
    lua_pushnumber(L, itemSlot->item.stackCount);  // count
    // Locked while its item is on the cursor: ContainerFrame_UpdateLockedItem
    // greys the slot from this, which is what shows the item has been picked up
    // out of it rather than still sitting there.
    const auto& held = cursorItemSlot();
    lua_pushboolean(L, (!held.equipped && held.bag == container && held.slot == slot) ? 1 : 0);
    lua_pushnumber(L, info ? info->quality : 0);  // quality
    lua_pushboolean(L, 0);  // readable
    lua_pushboolean(L, 0);  // lootable
    // Build item link with quality color
    std::string name = info ? info->name : ("Item #" + std::to_string(itemSlot->item.itemId));
    uint32_t q = info ? info->quality : 0;

    uint32_t qi = q < 8 ? q : 1u;
    const std::string link = game::itemChatLink(itemSlot->item.itemId, qi, name);
    lua_pushstring(L, link.c_str());  // link
    return 7;
}

// GetContainerItemLink(container, slot) → item link string
static int lua_GetContainerItemLink(lua_State* L) {
    auto* gh = getGameHandler(L);
    int container = static_cast<int>(luaL_checknumber(L, 1));
    int slot = static_cast<int>(luaL_checknumber(L, 2));
    if (!gh) { return luaReturnNil(L); }

    const auto& inv = gh->getInventory();
    // One lookup for every container the interface can name, the keyring
    // included: this branched on 0 and 1-4 and let KEYRING_CONTAINER fall
    // through, so the keyring frame opened with nothing in it.
    const game::ItemSlot* itemSlot = containerItemSlot(inv, container, slot);

    if (!itemSlot || itemSlot->empty()) { return luaReturnNil(L); }
    const auto* info = gh->getItemInfo(itemSlot->item.itemId);
    std::string name = info ? info->name : ("Item #" + std::to_string(itemSlot->item.itemId));
    uint32_t q = info ? info->quality : 0;
    const std::string link = game::itemChatLink(itemSlot->item.itemId, q, name);
    lua_pushstring(L, link.c_str());
    return 1;
}

// GetContainerNumFreeSlots(container) → numFreeSlots, bagType
static int lua_GetContainerNumFreeSlots(lua_State* L) {
    auto* gh = getGameHandler(L);
    int container = static_cast<int>(luaL_checknumber(L, 1));
    if (!gh) { lua_pushnumber(L, 0); lua_pushnumber(L, 0); return 2; }

    const auto& inv = gh->getInventory();
    int freeSlots = 0;
    int totalSlots = 0;

    totalSlots = containerSlotCount(inv, container);
    for (int i = 1; i <= totalSlots; ++i) {
        const game::ItemSlot* sl = containerItemSlot(inv, container, i);
        if (sl && sl->empty()) ++freeSlots;
    }

    lua_pushnumber(L, freeSlots);
    lua_pushnumber(L, 0);  // bagType (0 = normal)
    return 2;
}

// --- Equipment Slot API ---
// WoW inventory slot IDs: 1=Head,2=Neck,3=Shoulders,4=Shirt,5=Chest,
// 6=Waist,7=Legs,8=Feet,9=Wrists,10=Hands,11=Ring1,12=Ring2,
// 13=Trinket1,14=Trinket2,15=Back,16=MainHand,17=OffHand,18=Ranged,19=Tabard

// GetInventorySlotInfo("slotName") → slotId, textureName, checkRelic
// Maps WoW slot names (e.g. "HeadSlot", "HEADSLOT") to inventory slot IDs
static int lua_GetInventorySlotInfo(lua_State* L) {
    const char* name = luaL_checkstring(L, 1);
    std::string slot(name);
    // Normalize: uppercase, strip trailing "SLOT" if present
    for (char& c : slot) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    if (slot.size() > 4 && slot.substr(slot.size() - 4) == "SLOT")
        slot = slot.substr(0, slot.size() - 4);

    // WoW inventory slots are 1-indexed
    struct SlotMap { const char* name; int id; const char* texture; };
    static const SlotMap mapping[] = {
        {"HEAD",          1,  "Interface\\PaperDoll\\UI-PaperDoll-Slot-Head"},
        {"NECK",          2,  "Interface\\PaperDoll\\UI-PaperDoll-Slot-Neck"},
        {"SHOULDER",      3,  "Interface\\PaperDoll\\UI-PaperDoll-Slot-Shoulder"},
        {"SHIRT",         4,  "Interface\\PaperDoll\\UI-PaperDoll-Slot-Shirt"},
        {"CHEST",         5,  "Interface\\PaperDoll\\UI-PaperDoll-Slot-Chest"},
        {"WAIST",         6,  "Interface\\PaperDoll\\UI-PaperDoll-Slot-Waist"},
        {"LEGS",          7,  "Interface\\PaperDoll\\UI-PaperDoll-Slot-Legs"},
        {"FEET",          8,  "Interface\\PaperDoll\\UI-PaperDoll-Slot-Feet"},
        {"WRIST",         9,  "Interface\\PaperDoll\\UI-PaperDoll-Slot-Wrists"},
        {"HANDS",        10,  "Interface\\PaperDoll\\UI-PaperDoll-Slot-Hands"},
        {"FINGER0",      11,  "Interface\\PaperDoll\\UI-PaperDoll-Slot-Finger"},
        {"FINGER1",      12,  "Interface\\PaperDoll\\UI-PaperDoll-Slot-Finger"},
        {"TRINKET0",     13,  "Interface\\PaperDoll\\UI-PaperDoll-Slot-Trinket"},
        {"TRINKET1",     14,  "Interface\\PaperDoll\\UI-PaperDoll-Slot-Trinket"},
        {"BACK",         15,  "Interface\\PaperDoll\\UI-PaperDoll-Slot-Chest"},
        {"MAINHAND",     16,  "Interface\\PaperDoll\\UI-PaperDoll-Slot-MainHand"},
        {"SECONDARYHAND",17,  "Interface\\PaperDoll\\UI-PaperDoll-Slot-SecondaryHand"},
        {"RANGED",       18,  "Interface\\PaperDoll\\UI-PaperDoll-Slot-Ranged"},
        {"TABARD",       19,  "Interface\\PaperDoll\\UI-PaperDoll-Slot-Tabard"},
        // The bag buttons along the main bar ask for these by name at load, and
        // paperdollframe.lua does it in an OnLoad - so a gap here does not just
        // lose the bags, it loses the file.
        {"BAG0",         20,  "Interface\\PaperDoll\\UI-PaperDoll-Slot-Bag"},
        {"BAG1",         21,  "Interface\\PaperDoll\\UI-PaperDoll-Slot-Bag"},
        {"BAG2",         22,  "Interface\\PaperDoll\\UI-PaperDoll-Slot-Bag"},
        {"BAG3",         23,  "Interface\\PaperDoll\\UI-PaperDoll-Slot-Bag"},
        {"AMMO",          0,  "Interface\\PaperDoll\\UI-PaperDoll-Slot-Ammo"},
    };
    for (const auto& m : mapping) {
        if (slot == m.name) {
            lua_pushnumber(L, m.id);
            lua_pushstring(L, m.texture);
            lua_pushboolean(L, m.id == 18 ? 1 : 0); // checkRelic: only ranged slot
            return 3;
        }
    }
    // nil rather than an error, which is what the real client returns. Raising
    // here takes down the whole file that asked, and a name we do not know is
    // a gap in the table above rather than a reason to lose an interface.
    LOG_WARNING("GetInventorySlotInfo: unknown slot ", name);
    lua_pushnil(L);
    return 1;
}

/// Which of the three lists a name refers to. The panel says "list", "owner"
/// or "bidder" and every call that acts on a row is relative to one of them.
/// Takes a view, not a string: every caller passes the `const char*` straight
/// off the Lua stack, and a `const std::string&` parameter built a temporary
/// from it on each call. The temporary was harmless - the reference returned
/// points into the handler, never into `which` - but it made the compiler warn
/// that it might dangle, and a warning nobody can act on is worse than the
/// allocation it was reporting.
static const game::AuctionListResult& auctionListFor(game::GameHandler* gh,
                                                     std::string_view which) {
    if (which == "owner")  return gh->getAuctionOwnerResults();
    if (which == "bidder") return gh->getAuctionBidderResults();
    return gh->getAuctionBrowseResults();
}

/// The panel's own state: which row is selected, and which bag slot is sitting
/// in the sell box. Neither is anything the client has an opinion about.
static int& auctionSelection() { static int sel = 0; return sel; }

/// The item sitting in the sell box, as the flat (container, slot) pair the
/// cursor speaks in. Kept in that form rather than as a backpack index so a
/// worn bag can hold the item too - the send takes a guid, and every container
/// can produce one.
struct AuctionSellSlot {
    bool    held = false;
    uint8_t bag  = 0;
    uint8_t slot = 0;
};
static AuctionSellSlot& auctionSellSlot() { static AuctionSellSlot s; return s; }

/// The item a wire (container, slot) pair names, and its guid. Null when the
/// pair names nothing this client is holding.
static const game::ItemSlot* auctionSellItemSlot(game::GameHandler* gh,
                                                 uint64_t* guidOut = nullptr) {
    if (guidOut) *guidOut = 0;
    const AuctionSellSlot& sel = auctionSellSlot();
    if (!gh || !sel.held) return nullptr;
    const auto& inv = gh->getInventory();
    if (sel.bag == 0xFF) {
        const int index = static_cast<int>(sel.slot) - game::slots::backpackWireSlot(0);
        if (index < 0 || index >= inv.getBackpackSize()) return nullptr;
        const auto& s = inv.getBackpackSlot(index);
        if (s.empty()) return nullptr;
        if (guidOut) *guidOut = gh->getBackpackItemGuid(index);
        return &s;
    }
    for (int b = 0; b < game::Inventory::NUM_BAG_SLOTS; ++b) {
        if (sel.bag != static_cast<uint8_t>(game::slots::wornBagContainer(b))) continue;
        if (sel.slot >= inv.getBagSize(b)) return nullptr;
        const auto& s = inv.getBagSlot(b, sel.slot);
        if (s.empty()) return nullptr;
        if (guidOut) *guidOut = gh->getBagItemGuid(b, sel.slot);
        return &s;
    }
    return nullptr;
}

/// GetInventoryItemCount(unit, slot) → how many are in that equipped slot.
///
/// Stackable equipped things - ammo, thrown weapons - and the bank window's
/// own bag buttons, which print the count on the button face. Answering
/// nothing left every one of those blank.
static int lua_GetInventoryItemCount(lua_State* L) {
    auto* gh = getGameHandler(L);
    const char* uid = luaL_optstring(L, 1, "player");
    const int slotId = static_cast<int>(luaL_optnumber(L, 2, 0));
    std::string uidStr(uid);
    toLowerInPlace(uidStr);
    // Not 1..19: the bank is addressed through this too - bankframe.lua asks
    // GetInventoryItemCount("player", BankButtonIDToInvSlotID(id)) for every
    // one of its twenty-eight slots, and those ids start at forty.
    if (!gh || uidStr != "player" || !inventorySlotItem(gh->getInventory(), slotId)) {
        lua_pushnumber(L, 0);
        return 1;
    }
    const auto& slot = *inventorySlotItem(gh->getInventory(), slotId);
    if (slot.empty()) { lua_pushnumber(L, 0); return 1; }
    // A single item reports a stack of one rather than zero, which is how the
    // count is drawn: zero would print nothing where the client prints nothing
    // for one either, but the two mean different things to a caller.
    lua_pushnumber(L, slot.item.stackCount > 0 ? slot.item.stackCount : 1);
    return 1;
}

/// The item entry worn in an equipment slot by a unit that is not the player.
///
/// Everything about another player's gear arrives through inspect, and it
/// arrives as bare item entries with no inventory behind them. The three
/// GetInventoryItem* bindings answered nil for any unit but "player", which the
/// interface reads as an empty slot - so the inspect paperdoll drew every
/// square blank however well equipped the target was.
///
/// Zero when there is nothing cached for that unit, which is the honest answer
/// before the inspect response arrives.
static uint32_t inspectedItemEntry(game::GameHandler* gh, const std::string& uid, int slotId) {
    if (!gh || slotId < 1 || slotId > 19) return 0;
    const uint64_t guid = resolveUnitGuid(gh, uid);
    if (!guid) return 0;
    auto& cache = gh->inspectedPlayerItemEntriesRef();
    auto it = cache.find(guid);
    if (it == cache.end()) return 0;
    const uint32_t entry = it->second[static_cast<size_t>(slotId - 1)];
    // The name and icon come from the item cache, which will not have an entry
    // this client has never seen; asking queues the query.
    if (entry) gh->ensureItemInfo(entry);
    return entry;
}

static int lua_GetInventoryItemLink(lua_State* L) {
    auto* gh = getGameHandler(L);
    const char* uid = luaL_optstring(L, 1, "player");
    int slotId = static_cast<int>(luaL_checknumber(L, 2));
    // The bank is named through these too, at forty and up.
    if (!gh) { return luaReturnNil(L); }
    std::string uidStr(uid);
    toLowerInPlace(uidStr);
    uint32_t itemId = 0;
    std::string name;
    uint32_t q = 1;
    if (uidStr == "player") {
        const auto& inv = gh->getInventory();
        const game::ItemSlot* sp = inventorySlotItem(inv, slotId);
        if (!sp) { return luaReturnNil(L); }
        const auto& slot = *sp;
        if (slot.empty()) { return luaReturnNil(L); }
        itemId = slot.item.itemId;
        const auto* info = gh->getItemInfo(itemId);
        name = info ? info->name : slot.item.name;
        q = info ? info->quality : static_cast<uint32_t>(slot.item.quality);
    } else {
        itemId = inspectedItemEntry(gh, uidStr, slotId);
        if (!itemId) { return luaReturnNil(L); }
        const auto* info = gh->getItemInfo(itemId);
        if (!info) { return luaReturnNil(L); }
        name = info->name;
        q = info->quality;
    }

    uint32_t qi = q < 8 ? q : 1u;
    const std::string link = game::itemChatLink(itemId, qi, name);
    lua_pushstring(L, link.c_str());
    return 1;
}

static int lua_GetInventoryItemID(lua_State* L) {
    auto* gh = getGameHandler(L);
    const char* uid = luaL_optstring(L, 1, "player");
    int slotId = static_cast<int>(luaL_checknumber(L, 2));
    // The bank is named through these too, at forty and up.
    if (!gh) { return luaReturnNil(L); }
    std::string uidStr(uid);
    toLowerInPlace(uidStr);
    if (uidStr != "player") {
        const uint32_t entry = inspectedItemEntry(gh, uidStr, slotId);
        if (!entry) { return luaReturnNil(L); }
        lua_pushnumber(L, entry);
        return 1;
    }

    const game::ItemSlot* sp = inventorySlotItem(gh->getInventory(), slotId);
    if (!sp || sp->empty()) { return luaReturnNil(L); }
    const auto& slot = *sp;
    lua_pushnumber(L, slot.item.itemId);
    return 1;
}

static int lua_GetInventoryItemTexture(lua_State* L) {
    auto* gh = getGameHandler(L);
    const char* uid = luaL_optstring(L, 1, "player");
    int slotId = static_cast<int>(luaL_checknumber(L, 2));
    // 1..19 is head through tabard; 20..23 are the four bag slots. The bag bar
    // buttons ask about those four, and stopping at 19 answered "no bag" for
    // every one of them.
    // Equipment and the bank both. bankframe.lua draws its twenty-eight slots
    // with GetInventoryItemTexture("player", BankButtonIDToInvSlotID(id)), and
    // those ids land at forty and up - so a bound of NUM_SLOTS answered nil for
    // every one and the bank read as empty while bankSlots_ held the items.
    if (!gh || !inventorySlotItem(gh->getInventory(), slotId)) { return luaReturnNil(L); }
    std::string uidStr(uid);
    toLowerInPlace(uidStr);
    if (uidStr != "player") {
        const uint32_t entry = inspectedItemEntry(gh, uidStr, slotId);
        const auto* info = entry ? gh->getItemInfo(entry) : nullptr;
        if (!info || !info->displayInfoId) { return luaReturnNil(L); }
        const std::string path = gh->getItemIconPath(info->displayInfoId);
        lua_pushstring(L, path.empty() ? "Interface\\Icons\\INV_Misc_QuestionMark"
                                       : path.c_str());
        return 1;
    }

    const auto& inv = gh->getInventory();
    const auto& slot = *inventorySlotItem(inv, slotId);
    if (slot.empty()) { return luaReturnNil(L); }

    // Nil here means "empty slot" to the interface: PaperDollItemSlotButton_Update
    // draws the slot's background art instead of an item. Returning it for a
    // slot that holds something is why every equipped item - the bags on the
    // bag bar, and every square of the character sheet - looked unequipped.
    uint32_t displayId = slot.item.displayInfoId;
    if (displayId == 0) {
        if (const auto* info = gh->getItemInfo(slot.item.itemId)) displayId = info->displayInfoId;
    }
    const std::string icon = displayId ? gh->getItemIconPath(displayId) : std::string();
    lua_pushstring(L, icon.empty() ? "Interface\\Icons\\INV_Misc_QuestionMark"
                                   : icon.c_str());
    return 1;
}

/// ContainerIDToInventoryID(bagID) → the equipment slot that bag is worn in.
///
/// Bags 1 through 4 are inventory slots 20 through 23. The bag portrait button
/// asks for this to put the bag's own tooltip on itself, and left undefined the
/// call answered nil, which asked for the tooltip of no slot at all.
static int lua_ContainerIDToInventoryID(lua_State* L) {
    const int bag = static_cast<int>(luaL_checknumber(L, 1));
    lua_pushnumber(L, bag + 19);
    return 1;
}

/// GetBagName(bagID) → the name of the bag in that slot, or nil for an empty
/// one. The backpack is bag 0 and has a fixed name; the interface uses this to
/// label the bag buttons and their tooltips.
static int lua_GetBagName(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int bag = static_cast<int>(luaL_optnumber(L, 1, 0));
    if (bag == 0) { lua_pushstring(L, "Backpack"); return 1; }
    if (!gh || bag < 1 || bag > 4) { return luaReturnNil(L); }
    // An empty bag slot has no size, which is how the interface knows not to
    // draw a bag there at all.
    if (gh->getInventory().getBagSize(bag - 1) == 0) { return luaReturnNil(L); }
    lua_pushstring(L, "Bag");
    return 1;
}

/// SetBagPortraitTexture(texture, bagID) - the bag's own icon on the frame
/// that opens it.
///
/// The backpack has no item behind it and keeps the pack icon. Bags 1 to 4 are
/// worn in equipment slots 20 to 23, so the icon is the equipped bag's own -
/// which is why every open bag used to wear the same generic pack.
static int lua_SetBagPortraitTexture(lua_State* L) {
    if (!lua_istable(L, 1)) return 0;
    const int bag = static_cast<int>(luaL_optnumber(L, 2, 0));

    std::string icon = "Interface\\Buttons\\Button-Backpack-Up";
    if (bag >= 1 && bag <= 4) {
        if (auto* gh = getGameHandler(L)) {
            const auto& slot = gh->getInventory().getEquipSlot(
                static_cast<game::EquipSlot>(static_cast<int>(game::EquipSlot::BAG1) + bag - 1));
            if (!slot.empty()) {
                uint32_t displayId = slot.item.displayInfoId;
                if (displayId == 0) {
                    if (const auto* info = gh->getItemInfo(slot.item.itemId)) {
                        displayId = info->displayInfoId;
                    }
                }
                const std::string resolved =
                    displayId ? gh->getItemIconPath(displayId) : std::string();
                if (!resolved.empty()) icon = resolved;
            }
        }
    }

    lua_getfield(L, 1, "SetTexture");
    if (!lua_isfunction(L, -1)) { lua_pop(L, 1); return 0; }
    lua_pushvalue(L, 1);
    lua_pushstring(L, icon.c_str());
    lua_call(L, 2, 0);
    return 0;
}

/// Where the cursor's item sits, in the numbering the server uses. The same
/// translation the pickup bindings do; here so the two "put it in a container"
/// calls below can reach it.
static bool heldWireSlot(uint8_t& bag, uint8_t& slot) {
    const auto& held = cursorItemSlot();
    if (held.bag < 0 && !held.equipped) return false;
    if (held.equipped) {
        bag = 0xFF;
        slot = static_cast<uint8_t>(held.slot - 1);
    } else if (held.bag == 0) {
        bag = 0xFF;
        slot = static_cast<uint8_t>(game::slots::backpackWireSlot(held.slot - 1));
    } else {
        bag = static_cast<uint8_t>(game::slots::wornBagContainer(held.bag - 1));
        slot = static_cast<uint8_t>(held.slot - 1);
    }
    return true;
}

/// PutItemInBag(inventoryID) - put what the cursor is holding into that bag.
///
/// The bag buttons along the bottom bar call this before deciding what a click
/// meant: an empty cursor answers false and the button opens the bag instead,
/// which is why a no-op behaved correctly for a plain click and did nothing at
/// all for a click that was carrying something.
static int lua_PutItemInBag(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int inventoryId = static_cast<int>(luaL_optnumber(L, 1, 0));
    uint8_t srcBag = 0, srcSlot = 0;
    if (!gh || inventoryId < 20 || inventoryId > 23 || !heldWireSlot(srcBag, srcSlot)) {
        lua_pushboolean(L, 0);
        return 1;
    }
    const int bagIndex = inventoryId - 20;          // 0-3
    const auto& inv = gh->getInventory();
    const int size = inv.getBagSize(bagIndex);
    // An empty bag slot equips the held bag rather than swallowing it.
    //
    // getBagSize is zero when nothing is worn in this slot, so the loop below
    // - which finds a free content slot inside the equipped bag - had nothing
    // to iterate and fell straight through, doing nothing. Dropping a bag onto
    // an empty bag slot therefore looked dead. This is the equip: swap the held
    // item into the worn-bag equipment slot, the same move PickupBagFromSlot's
    // own drop makes, with 0xFF for the equipment container.
    if (size == 0) {
        gh->swapContainerItems(srcBag, srcSlot, 0xFF,
                               static_cast<uint8_t>(game::slots::toWireSlot(inventoryId)));
        cursorItemSlot() = {};
        wowee::ui::frameXmlSetCursorItem(std::string());
        lua_pushboolean(L, 1);
        return 1;
    }
    for (int i = 0; i < size; ++i) {
        if (!inv.getBagSlot(bagIndex, i).empty()) continue;
        gh->swapContainerItems(srcBag, srcSlot,
                               static_cast<uint8_t>(game::slots::wornBagContainer(bagIndex)),
                               static_cast<uint8_t>(i));
        cursorItemSlot() = {};
        wowee::ui::frameXmlSetCursorItem(std::string());
        lua_pushboolean(L, 1);
        return 1;
    }
    // Held, but nowhere to put it - still "had an item", so the button does not
    // fall through to opening the bag.
    lua_pushboolean(L, 1);
    return 1;
}

/// PutItemInBackpack() - the same, for the backpack.
static int lua_PutItemInBackpack(lua_State* L) {
    auto* gh = getGameHandler(L);
    uint8_t srcBag = 0, srcSlot = 0;
    if (!gh || !heldWireSlot(srcBag, srcSlot)) {
        lua_pushboolean(L, 0);
        return 1;
    }
    const int free = gh->getInventory().findFreeBackpackSlot();
    if (free >= 0) {
        gh->swapContainerItems(srcBag, srcSlot, 0xFF,
                               static_cast<uint8_t>(game::slots::backpackWireSlot(free)));
        cursorItemSlot() = {};
        wowee::ui::frameXmlSetCursorItem(std::string());
    }
    lua_pushboolean(L, 1);
    return 1;
}

/// ResetCursor() - put the pointer back to the ordinary arrow. This client
/// does not change the cursor for interface state, so there is nothing to
/// undo; it exists because the interface calls it on every mouse-leave.
static int lua_ResetCursor(lua_State* L) { (void)L; return 0; }

/// Coin as words, for the name of the money slot: "1 Gold, 20 Silver".
/// Empty denominations are left out rather than shown as zero.
static std::string lootCoinText(uint32_t copper) {
    const uint32_t g = copper / 10000, s = (copper / 100) % 100, c = copper % 100;
    std::string out;
    auto add = [&out](uint32_t v, const char* unit) {
        if (v == 0) return;
        if (!out.empty()) out += ", ";
        out += std::to_string(v);
        out += ' ';
        out += unit;
    };
    add(g, "Gold");
    add(s, "Silver");
    add(c, "Copper");
    return out;
}

/// Money occupies a loot slot in the interface but not on the wire.
///
/// The real client shows coin as the first slot when there is any, with the
/// items after it, and the loot frame asks LootSlotIsCoin which one it is
/// looking at. The server numbers only the items, so every slot here is
/// translated before it is sent - which LootSlot already did by carrying
/// LootItem::slotIndex rather than the display position.
static bool lootHasCoin(game::GameHandler* gh) {
    return gh && gh->isLootWindowOpen() && gh->getCurrentLoot().gold > 0;
}

/// The item behind a display slot, or null when the slot is the coin or past
/// the end.
static const game::LootItem* lootItemAtSlot(game::GameHandler* gh, int slot) {
    if (!gh || !gh->isLootWindowOpen() || slot < 1) return nullptr;
    const auto& loot = gh->getCurrentLoot();
    const int itemIndex = lootHasCoin(gh) ? slot - 1 : slot;   // 1-based already
    if (itemIndex < 1 || itemIndex > static_cast<int>(loot.items.size())) return nullptr;
    return &loot.items[itemIndex - 1];
}

// LootSlotIsCoin(slot) → whether this slot is the money
static int lua_LootSlotIsCoin(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int slot = static_cast<int>(luaL_optnumber(L, 1, 0));
    lua_pushboolean(L, (lootHasCoin(gh) && slot == 1) ? 1 : 0);
    return 1;
}

// LootSlotIsItem(slot) → whether this slot is a real item
static int lua_LootSlotIsItem(lua_State* L) {
    lua_pushboolean(L, lootItemAtSlot(getGameHandler(L),
                                      static_cast<int>(luaL_optnumber(L, 1, 0))) ? 1 : 0);
    return 1;
}

// IsFishingLoot() → whether this came out of the water
static int lua_IsFishingLoot(lua_State* L) {
    auto* gh = getGameHandler(L);
    // LOOT_TYPE_FISHING, as the server sends it.
    const bool fishing = gh && gh->isLootWindowOpen() &&
                         gh->getCurrentLoot().lootType == 2;
    lua_pushboolean(L, fishing ? 1 : 0);
    return 1;
}

static int lua_GetNumLootItems(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh || !gh->isLootWindowOpen()) { return luaReturnZero(L); }
    // Coin counts as a slot, which is what the loot frame iterates over.
    lua_pushnumber(L, gh->getCurrentLoot().items.size() + (lootHasCoin(gh) ? 1 : 0));
    return 1;
}

// GetLootSlotInfo(slot) → texture, name, quantity, quality, locked
static int lua_GetLootSlotInfo(lua_State* L) {
    auto* gh = getGameHandler(L);
    int slot = static_cast<int>(luaL_checknumber(L, 1)); // 1-indexed
    if (!gh || !gh->isLootWindowOpen()) {
        return luaReturnNil(L);
    }
    const auto& loot = gh->getCurrentLoot();
    if (lootHasCoin(gh) && slot == 1) {
        // The coin slot describes itself: the interface shows the amount as the
        // name and has a texture of its own for it.
        lua_pushstring(L, "Interface\\Icons\\INV_Misc_Coin_01");
        lua_pushstring(L, lootCoinText(loot.gold).c_str());
        lua_pushnumber(L, loot.gold);
        lua_pushnumber(L, 1);
        lua_pushboolean(L, 0);
        return 5;
    }
    const auto* itemPtr = lootItemAtSlot(gh, slot);
    if (!itemPtr) { return luaReturnNil(L); }
    const auto& item = *itemPtr;
    const auto* info = gh->getItemInfo(item.itemId);

    // texture (icon path from ItemDisplayInfo.dbc)
    std::string icon;
    if (info && info->displayInfoId != 0) {
        icon = gh->getItemIconPath(info->displayInfoId);
    }
    if (!icon.empty()) lua_pushstring(L, icon.c_str());
    else lua_pushnil(L);

    // name
    if (info && !info->name.empty()) lua_pushstring(L, info->name.c_str());
    else lua_pushstring(L, ("Item #" + std::to_string(item.itemId)).c_str());

    lua_pushnumber(L, item.count);                           // quantity
    lua_pushnumber(L, info ? info->quality : 1);             // quality
    // locked - the row the loot window draws in red and refuses to take.
    //
    // Answered false because the slot type was "not tracked". It is tracked:
    // SMSG_LOOT_RESPONSE carries one per item and the parser has always stored
    // it. AzerothCore's LootSlotType is ALLOW_LOOT 0, ROLL_ONGOING 1, MASTER 2,
    // LOCKED 3, OWNER 4 - so everything except the two that mean "take it" is
    // locked, which covers a roll still running and an item only the master
    // looter can hand out. Both used to draw as ordinary loot that silently did
    // nothing when clicked.
    constexpr uint8_t kAllowLoot = 0, kOwner = 4;
    const bool locked = item.lootSlotType != kAllowLoot &&
                        item.lootSlotType != kOwner;
    lua_pushboolean(L, locked ? 1 : 0);
    return 5;
}

// GetLootSlotLink(slot) → itemLink
static int lua_GetLootSlotLink(lua_State* L) {
    auto* gh = getGameHandler(L);
    int slot = static_cast<int>(luaL_checknumber(L, 1));
    if (!gh || !gh->isLootWindowOpen()) { return luaReturnNil(L); }
    const auto* itemPtr = lootItemAtSlot(gh, slot);
    if (!itemPtr) { return luaReturnNil(L); }   // coin has no link
    const auto& item = *itemPtr;
    const auto* info = gh->getItemInfo(item.itemId);
    if (!info || info->name.empty()) { return luaReturnNil(L); }

    uint32_t qi = info->quality < 8 ? info->quality : 1u;
    const std::string link = game::itemChatLink(item.itemId, qi, info->name);
    lua_pushstring(L, link.c_str());
    return 1;
}

// LootSlot(slot) - take item from loot
static int lua_LootSlot(lua_State* L) {
    auto* gh = getGameHandler(L);
    int slot = static_cast<int>(luaL_checknumber(L, 1));
    if (!gh || !gh->isLootWindowOpen()) return 0;
    if (lootHasCoin(gh) && slot == 1) { gh->lootMoney(); return 0; }
    if (const auto* item = lootItemAtSlot(gh, slot)) {
        // The server's own slot number, not the position on screen.
        gh->lootItem(item->slotIndex);
    }
    return 0;
}

// CloseLoot() - close loot window
static int lua_CloseLoot(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (gh) gh->closeLoot();
    return 0;
}

// --- Group loot rolls ---
//
// The roll window opens on START_LOOT_ROLL, which this client fires, and then
// asked three questions it had no answer to: what is being rolled for, how long
// is left, and - when a button was pressed - nothing happened, because
// RollOnLoot did not exist. So the window appeared over a roll the player could
// watch and not take part in, and passed by default when the timer ran out on
// the server.
//
// All of it was already here. SMSG_LOOT_START_ROLL is parsed into a
// LootRollEntry with the item, the quality, the countdown and the mask of which
// buttons the server will accept, and sendLootRoll addresses the reply by the
// object and slot the roll came from.
//
// One roll at a time is all this client keeps, which is the same limit its own
// window has. The roll id is the loot slot plus one - stable for the length of
// the roll, never zero, and checked rather than trusted, so a frame left over
// from a previous roll is told there is nothing there and hides itself instead
// of answering about the wrong item.
namespace {

/// The roll a Lua roll id refers to, or null if it is not the one in progress.
const game::LootRollEntry* rollFor(game::GameHandler* gh, int rollId) {
    if (!gh || !gh->hasPendingLootRoll()) return nullptr;
    const auto& roll = gh->getPendingLootRoll();
    if (rollId != static_cast<int>(roll.slot) + 1) return nullptr;
    return &roll;
}

// Which buttons the server said it would accept. The same three bits the
// roll packet carries.
constexpr uint8_t kRollNeed       = 0x01;
constexpr uint8_t kRollGreed      = 0x02;
constexpr uint8_t kRollDisenchant = 0x04;

} // namespace

// GetLootRollItemInfo(rollId) → texture, name, count, quality, bindOnPickUp,
//   canNeed, canGreed, canDisenchant, reasonNeed, reasonGreed,
//   reasonDisenchant, deSkillRequired
//
// All twelve, because the roll window unpacks all twelve and uses the tail of
// them: a reason is concatenated into a global's name when its button is
// disabled, and the count is compared against one. Nil in either place raises.
// The reasons are the generic "your class may not" line - this client is not
// told why the server refused a button, only that it did.
static int lua_GetLootRollItemInfo(lua_State* L) {
    auto* gh = getGameHandler(L);
    const auto* roll = rollFor(gh, static_cast<int>(luaL_optnumber(L, 1, 0)));
    if (!roll) { return luaReturnNil(L); }

    const auto* info = gh->getItemInfo(roll->itemId);
    lua_pushstring(L, gh->getItemIconPath(info ? info->displayInfoId : 0).c_str());
    lua_pushstring(L, roll->itemName.c_str());
    // The roll packet does not carry a stack size. One is right for nearly
    // everything rolled for, and the window only reads it to decide whether to
    // print a number in the corner at all.
    lua_pushnumber(L, 1);
    lua_pushnumber(L, roll->itemQuality);
    lua_pushboolean(L, info && info->bindType == 1 ? 1 : 0);  // bind on pickup
    lua_pushboolean(L, (roll->voteMask & kRollNeed) ? 1 : 0);
    lua_pushboolean(L, (roll->voteMask & kRollGreed) ? 1 : 0);
    lua_pushboolean(L, (roll->voteMask & kRollDisenchant) ? 1 : 0);
    lua_pushnumber(L, 1);   // reasonNeed        → "Your class may not roll need"
    lua_pushnumber(L, 1);   // reasonGreed
    lua_pushnumber(L, 3);   // reasonDisenchant  → "may not be disenchanted"
    lua_pushnumber(L, 0);   // disenchanting skill the group would need
    return 12;
}

static int lua_GetLootRollItemLink(lua_State* L) {
    auto* gh = getGameHandler(L);
    const auto* roll = rollFor(gh, static_cast<int>(luaL_optnumber(L, 1, 0)));
    if (!roll) { return luaReturnNil(L); }
    lua_pushstring(L, game::buildItemLink(roll->itemId, roll->itemQuality,
                                          roll->itemName).c_str());
    return 1;
}

// GetLootRollTimeLeft(rollId) → milliseconds still to answer in.
//
// Answered zero before, from the list of counts that are genuinely nothing.
// It is not nothing here: the bar under the item is drawn from it, so it sat
// empty for the whole roll.
static int lua_GetLootRollTimeLeft(lua_State* L) {
    auto* gh = getGameHandler(L);
    const auto* roll = rollFor(gh, static_cast<int>(luaL_optnumber(L, 1, 0)));
    if (!roll) { lua_pushnumber(L, 0); return 1; }
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - roll->rollStartedAt).count();
    const auto left = static_cast<int64_t>(roll->rollCountdownMs) - elapsed;
    lua_pushnumber(L, left > 0 ? static_cast<double>(left) : 0.0);
    return 1;
}

// RollOnLoot(rollId, rollType) - 0 pass, 1 need, 2 greed, 3 disenchant, which
// is both the order the buttons carry as their id and the order the server
// expects.
static int lua_RollOnLoot(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int rollId = static_cast<int>(luaL_optnumber(L, 1, 0));
    const auto* roll = rollFor(gh, rollId);
    if (!roll) return 0;
    const int type = static_cast<int>(luaL_optnumber(L, 2, 0));
    if (type < 0 || type > 3) return 0;

    // Rolling for something that will bind asks first. CONFIRM_LOOT_ROLL is
    // raised by the client rather than the server - uiparent.lua answers it
    // with the popup whose OK calls ConfirmLootRoll(id, rollType) - so nothing
    // raised it here and Need on a bind-on-pickup item bound it with no
    // warning at all, which is the one place this differs from the real client
    // in a way a player pays for.
    //
    // Need and Greed only. Both hand the item over and bind it; disenchant
    // yields the shard and passing yields nothing, so neither asks.
    if (type == 1 || type == 2) {
        const auto* info = gh->getItemInfo(roll->itemId);
        if (info && info->valid && info->bindType == 1) {
            gh->fireAddonEvent("CONFIRM_LOOT_ROLL",
                               {std::to_string(rollId), std::to_string(type)});
            return 0;
        }
    }

    gh->sendLootRoll(roll->objectGuid, roll->slot, static_cast<uint8_t>(type));
    return 0;
}

// ConfirmLootRoll(rollId, rollType) - the answer to the popup above, which is
// the only caller. Sends without asking again, because this *is* the asking.
static int lua_ConfirmLootRoll(lua_State* L) {
    auto* gh = getGameHandler(L);
    const auto* roll = rollFor(gh, static_cast<int>(luaL_optnumber(L, 1, 0)));
    if (!roll) return 0;
    const int type = static_cast<int>(luaL_optnumber(L, 2, 0));
    if (type < 0 || type > 3) return 0;
    gh->sendLootRoll(roll->objectGuid, roll->slot, static_cast<uint8_t>(type));
    return 0;
}

// GiveMasterLoot(slot, candidate) - hand an item to someone, as master looter.
//
// The candidate is a position in the list the server sent with the loot, not a
// guid: the menu is built by walking that list, and the entry clicked is the
// number passed back. Checked against the list rather than trusted, since a
// menu left open while the loot changed would otherwise name whoever now
// happens to sit at that position.
static int lua_GiveMasterLoot(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int slot = static_cast<int>(luaL_optnumber(L, 1, 0));
    const int candidate = static_cast<int>(luaL_optnumber(L, 2, 0));
    if (!gh || slot < 1 || candidate < 1) return 0;

    const auto& candidates = gh->getMasterLootCandidates();
    if (candidate > static_cast<int>(candidates.size())) return 0;
    gh->lootMasterGive(static_cast<uint8_t>(slot - 1),
                       candidates[static_cast<size_t>(candidate - 1)]);
    return 0;
}

// GetMasterLootCandidate(index) → the name of whoever can be given this loot.
//
// The list the server sends with the loot, which GiveMasterLoot has been
// reading all along - only the call that *builds* the menu from it was a stub.
// So a master looter got an empty "Give loot" submenu over a list the client
// had, and the function that would have acted on a choice was finished and
// unreachable.
//
// Nil for an index past the end, which is how GroupLootDropDown_Initialize
// stops: it walks five at a time and adds a button only `if ( candidate )`.
static int lua_GetMasterLootCandidate(lua_State* L) {
    auto* gh = getGameHandler(L);
    const int index = static_cast<int>(luaL_optnumber(L, 1, 0));
    if (!gh || index < 1) return luaReturnNil(L);
    const auto& candidates = gh->getMasterLootCandidates();
    if (index > static_cast<int>(candidates.size())) return luaReturnNil(L);

    const uint64_t guid = candidates[static_cast<size_t>(index) - 1];
    std::string name = gh->lookupName(guid);
    // A name this client has not learned yet would come back empty, and an
    // empty string is true in Lua - the menu would show a blank row that gives
    // the item to someone unnamed. Nil skips the row instead.
    if (name.empty()) return luaReturnNil(L);
    lua_pushstring(L, name.c_str());
    return 1;
}

// GetLootMethod() → "freeforall"|"roundrobin"|"master"|"group"|"needbeforegreed", partyLoot, raidLoot
static int lua_GetLootMethod(lua_State* L) {
    auto* gh = getGameHandler(L);
    // Nil for the two indices here as well: no handler is no group, and a
    // zero would claim the player is the master looter of it.
    if (!gh) { lua_pushstring(L, "freeforall"); lua_pushnil(L); lua_pushnil(L); return 3; }
    const auto& pd = gh->getPartyData();
    lua_pushstring(L, pd.lootMethod < kNumLootMethods
                          ? kLootMethodTokens[pd.lootMethod]
                          : kLootMethodTokens[0]);
    // Who the master looter is, or nobody.
    //
    // Zero is not "nobody" here - it is *the player*. playerframe.lua reads
    //     if ( lootMaster == 0 and (in a party or raid) ) then
    //         PlayerMasterIcon:Show()
    // so answering zero unconditionally hung the master-looter crown on the
    // player's own frame in every group, whatever the loot method was and
    // whoever was actually holding it. partymemberframe.lua does the mirror
    // of that with `if ( id == lootMaster )`.
    //
    // Nil is how the client says there is no master looter, and it is the
    // truthful answer under every method but master loot. The guid comes with
    // the group list, so when there is one it can be named rather than
    // guessed: index 0 is the player, 1 upward the party members in order.
    static constexpr uint8_t kMasterLoot = 2;
    if (pd.lootMethod != kMasterLoot || pd.looterGuid == 0) {
        lua_pushnil(L);
        lua_pushnil(L);
    } else if (pd.looterGuid == gh->getPlayerGuid()) {
        lua_pushnumber(L, 0);
        lua_pushnil(L);
    } else {
        int partyIndex = -1;
        for (size_t i = 0; i < pd.members.size(); ++i) {
            if (pd.members[i].guid == pd.looterGuid) {
                partyIndex = static_cast<int>(i) + 1;
                break;
            }
        }
        if (partyIndex > 0) lua_pushnumber(L, partyIndex);
        else                lua_pushnil(L);
        // The raid index is a different numbering and this client does not
        // keep one, so it says so rather than reusing the party position.
        lua_pushnil(L);
    }
    return 3;
}

// --- Additional WoW API ---

static int lua_GetItemLink(lua_State* L) {
    auto* gh = getGameHandler(L);
    if (!gh) { return luaReturnNil(L); }
    uint32_t itemId = static_cast<uint32_t>(luaL_checknumber(L, 1));
    if (itemId == 0) { return luaReturnNil(L); }
    const auto* info = gh->getItemInfo(itemId);
    if (!info || info->name.empty()) { return luaReturnNil(L); }

    uint32_t qi = info->quality < 8 ? info->quality : 1u;
    const std::string link = game::itemChatLink(itemId, qi, info->name);
    lua_pushstring(L, link.c_str());
    return 1;
}

// GetSpellLink(spellIdOrName) → "|cFFxxxxxx|Hspell:ID|h[Name]|h|r"

void registerInventoryLuaAPI(lua_State* L) {
    static const struct { const char* name; lua_CFunction func; } api[] = {
                {"GetMoney",      lua_GetMoney},
                // The money cursor. A drag of money is routed entirely by the
                // interface - the frame it lands on reads the amount, puts it
                // where it belongs, and clears the cursor - so the client's
                // whole part is holding the number. It held nothing, so money
                // could not be dragged into a mail, a trade, a guild bank
                // deposit or an auction bid.
                {"GetCursorMoney", [](lua_State* L) -> int {
            lua_pushnumber(L, static_cast<lua_Number>(cursorMoney()));
            return 1;
        }},
                {"DropCursorMoney", [](lua_State* L) -> int {
            setCursorMoney(L, 0);
            return 0;
        }},
                {"GetPlayerTradeMoney", lua_GetPlayerTradeMoney},
                {"GetTargetTradeMoney", lua_GetTargetTradeMoney},
                {"GetMerchantNumItems",  lua_GetMerchantNumItems},
                {"GetMerchantItemInfo",  lua_GetMerchantItemInfo},
                {"GetMerchantItemLink",  lua_GetMerchantItemLink},
                {"CanMerchantRepair",    lua_CanMerchantRepair},
                {"GetContainerItemCooldown",  lua_GetContainerItemCooldown},
                {"GetBankSlotCost",        lua_GetBankSlotCost},
                {"GetNumBankSlots",        lua_GetNumBankSlots},
                {"GetInventoryAlertStatus",   lua_GetInventoryAlertStatus},
                {"GetContainerItemQuestInfo", lua_GetContainerItemQuestInfo},
                {"KeyRingButtonIDToInvSlotID", lua_KeyRingButtonIDToInvSlotID},
                {"SetPortraitToTexture",  lua_SetPortraitToTexture},
                // The "you can't do that when you're dead" line, which
                // containerframe.lua and uiparent.lua raise when a panel is
                // asked for while dead. A no-op meant the bags simply refused
                // to open with nothing said about why.
                {"NotWhileDeadError", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L)) {
                gh->addUIError("You can't do that when you're dead.");
            }
            return 0;
        }},
                {"ShowContainerSellCursor", lua_ContainerNoOp},
                {"ShowBuybackSellCursor", lua_ContainerNoOp},
                // A left-click on a vendor's item goes here, and it was a
                // no-op - so at a merchant only right-click bought anything.
                // The cursor state lives in lua_action_api.cpp with everything
                // else that picks up and puts down, so this hands off to it.
                {"PickupMerchantItem", [](lua_State* L) -> int {
            pickupMerchantItem(L, static_cast<int>(luaL_optnumber(L, 1, 0)));
            return 0;
        }},
                // SocketInventoryItem and SocketContainerItem used to answer
                // false here, on the reading that per-item socket contents were
                // not tracked so the panel was better left shut than opened
                // empty. They are tracked - the item's enchantment fields carry
                // them - and both live in lua_socket_api.cpp now with the rest
                // of the socketing surface. Two registrations of one name would
                // be settled by load order.
                {"SpellCanTargetItem",    lua_SpellCanTargetItem},
                // CanGuildBankRepair() - whether the guild-repair button
                // appears beside the merchant's own.
                //
                // Answered false while the rank rights were being read wrongly;
                // everything it needs has been here since. The right is
                // GR_RIGHT_WITHDRAW_REPAIR from AzerothCore's GuildRankRights,
                // held by the player's own rank, and the bank has to have
                // something in it - the button offers to spend guild money, and
                // offering to spend none is offering nothing.
                //
                // RepairAllItems already sends the guild flag, so this gate was
                // the whole of what was missing.
                {"CanGuildBankRepair", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh || gh->getGuildBankData().money == 0) { return luaReturnFalse(L); }
            const auto& roster = gh->getGuildRoster();
            const uint64_t self = gh->getPlayerGuid();
            for (const auto& m : roster.members) {
                if (m.guid != self) continue;
                if (m.rankIndex >= roster.ranks.size()) break;
                constexpr uint32_t kWithdrawRepair = 0x00040000u;
                const bool may =
                    (roster.ranks[m.rankIndex].rights & kWithdrawRepair) != 0;
                lua_pushboolean(L, may ? 1 : 0);
                return 1;
            }
            return luaReturnFalse(L);
        }},
                {"GetBuybackItemInfo",      lua_GetBuybackItemInfo},
                {"GetBuybackItemLink",      lua_GetBuybackItemLink},
                {"BuybackItem",             lua_BuybackItem},
                {"GetRepairAllCost",        lua_GetRepairAllCost},
                {"CloseMerchant",           lua_CloseMerchant},
                {"GetMerchantItemMaxStack", lua_GetMerchantItemMaxStack},
                {"GetMerchantItemCostInfo", lua_GetMerchantItemCostInfo},
                {"GetMerchantItemCostItem", lua_GetMerchantItemCostItem},
                {"GetItemInfo",       lua_GetItemInfo},
                {"__WoweeTryOn",   lua_WoweeTryOn},
                {"__WoweeSetModelCreature", lua_WoweeSetModelCreature},
                {"__WoweeUndress", lua_WoweeUndress},
                {"IsDressableItem",   lua_IsDressableItem},
                {"GetItemQualityColor", lua_GetItemQualityColor},
                {"_GetItemTooltipData", lua_GetItemTooltipData},
                // GetItemSpell(item) → spellName, spellRank
                //
                // The "Use:" spell on an item, which is what /use and the chat
                // macro parser look for to tell a usable item from an inert
                // one. Trigger 0 is on-use; equip and proc effects are not what
                // is being asked for.
                {"GetItemSpell", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) return luaReturnNil(L);
            // Either an item id or a name, as everywhere else an item is named.
            uint32_t itemId = 0;
            if (lua_isnumber(L, 1)) {
                itemId = static_cast<uint32_t>(lua_tonumber(L, 1));
            } else if (const char* s = lua_tostring(L, 1)) {
                std::string str(s);
                // An item link carries the id; a bare name has to be matched.
                const auto pos = str.find("item:");
                if (pos != std::string::npos) {
                    itemId = static_cast<uint32_t>(std::strtoul(str.c_str() + pos + 5, nullptr, 10));
                } else {
                    for (const auto& [id, info] : gh->getItemInfoCache()) {
                        if (info.name == str) { itemId = id; break; }
                    }
                }
            }
            if (itemId == 0) return luaReturnNil(L);
            const auto* info = gh->getItemInfo(itemId);
            if (!info) return luaReturnNil(L);
            for (const auto& sp : info->spells) {
                if (sp.spellId == 0 || sp.spellTrigger != 0) continue;
                const std::string& name = gh->getSpellName(sp.spellId);
                if (name.empty()) continue;
                lua_pushstring(L, name.c_str());
                // The spell's rank subtext, the same string the spellbook shows
                // - empty for the many item spells that carry no rank, the real
                // "Rank N" for those that do. It is tracked; it was hardcoded "".
                lua_pushstring(L, gh->getSpellRank(sp.spellId).c_str());
                return 2;
            }
            return luaReturnNil(L);
        }},
                // ---- Currency tab ----
                {"GetContainerItemPurchaseInfo", lua_GetContainerItemPurchaseInfo},
                {"GetContainerItemPurchaseItem", lua_GetContainerItemPurchaseItem},
                // ContainerRefundItemPurchase(bag, slot) - the Accept on
                // "hand this back for what you paid". It sat on a popup that
                // could not be raised, because the call above answered nil.
                {"ContainerRefundItemPurchase", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int bag = static_cast<int>(luaL_optnumber(L, 1, 0));
            const int slot = static_cast<int>(luaL_optnumber(L, 2, 0));
            if (gh) gh->refundItem(containerSlotGuid(gh, bag, slot));
            return 0;
        }},
                // PickupPlayerMoney(amount) - the coin pickup dialog's Okay.
                // Clamped to what the player has: the dialog does not check,
                // and a cursor carrying more than the purse would be spent by
                // whatever it was dropped on.
                {"PickupPlayerMoney", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const double asked = luaL_optnumber(L, 1, 0);
            if (!gh || asked <= 0) { setCursorMoney(L, 0); return 0; }
            const uint64_t have = gh->getMoneyCopper();
            const uint64_t want = static_cast<uint64_t>(asked);
            setCursorMoney(L, want < have ? want : have);
            return 0;
        }},
                {"PickupTradeMoney",        lua_PickupTradeMoney},
                {"PickupSendMailMoney",     lua_PickupSendMailMoney},
                {"PickupSendMailCOD",       lua_MoneyCursorNoop},
                {"PickupGuildBankMoney",    lua_MoneyCursorNoop},
                {"AddTradeMoney",           lua_AddTradeMoney},
                {"GetCurrencyListSize", [](lua_State* L) -> int {
            lua_pushnumber(L, static_cast<lua_Number>(buildCurrencyList(L).size()));
            return 1;
        }},
                // GetCurrencyListInfo(index) → name, isHeader, isExpanded,
                //   isUnused, isWatched, count, extraCurrencyType, icon, itemID
                {"GetCurrencyListInfo", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int idx = static_cast<int>(luaL_optnumber(L, 1, 0));
            const auto rows = buildCurrencyList(L);
            if (idx < 1 || idx > static_cast<int>(rows.size())) return luaReturnNil(L);
            const auto& r = rows[static_cast<size_t>(idx) - 1];
            lua_pushstring(L, r.name.c_str());   // 1: name
            lua_pushboolean(L, 0);               // 2: isHeader - the list is flat
            lua_pushboolean(L, 0);               // 3: isExpanded
            lua_pushboolean(L, 0);               // 4: isUnused
            lua_pushboolean(L, 0);               // 5: isWatched
            lua_pushnumber(L, r.count);          // 6: count
            lua_pushnumber(L, 0);                // 7: extraCurrencyType
            // A nil texture is an empty slot to the interface, and
            // TokenFrame_Update draws the row's icon from this - so every
            // currency was listed with a blank square beside its name. The
            // icon is the item's, which is where a currency's art lives.
            const auto* info = gh ? gh->getItemInfo(r.itemId) : nullptr;
            const std::string icon = (info && info->displayInfoId)
                ? gh->getItemIconPath(info->displayInfoId) : std::string();
            if (icon.empty()) lua_pushstring(L, "Interface\\Icons\\INV_Misc_QuestionMark");
            else              lua_pushstring(L, icon.c_str());   // 8: icon
            lua_pushnumber(L, r.itemId);         // 9: itemID
            return 9;
        }},
                // GetBackpackCurrencyInfo(index) → name, count, icon, currencyTypesID
                //
                // Nothing is pinned to the backpack: that is a saved choice the
                // client does not keep, and answering with the whole list would
                // put every currency under the bags.
                {"GetBackpackCurrencyInfo", [](lua_State* L) -> int { return luaReturnNil(L); }},
                // The three that change how the list is displayed. Each is a
                // saved preference with nowhere to be saved, so they are
                // accepted and forgotten rather than left to raise.
                {"ExpandCurrencyList",  [](lua_State* L) -> int { (void)L; return 0; }},
                {"SetCurrencyBackpack", [](lua_State* L) -> int { (void)L; return 0; }},
                {"SetCurrencyUnused",   [](lua_State* L) -> int { (void)L; return 0; }},
                {"GetItemCount",      lua_GetItemCount},
                {"UseContainerItem",  lua_UseContainerItem},
                // SortBags() - merge partial stacks, then order every bag slot.
                //
                // Not a 3.3.5 function: sorting arrived years later, so nothing
                // in FrameXML calls this. It is here for the bundled all-bags
                // addon, and it is the same sort this client's own bag window
                // has always had rather than a second implementation of it.
                {"SortBags", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L)) gh->sortBags();
            return 0;
        }},
                // Whether that sort is still sending its moves, so a button can
                // say so rather than looking dead while dozens of swaps go out
                // a tick at a time.
                {"IsSortingBags", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushboolean(L, (gh && gh->isSortingItems()) ? 1 : 0);
            return 1;
        }},
                {"GetContainerNumSlots",    lua_GetContainerNumSlots},
                {"GetKeyRingSize",          lua_GetKeyRingSize},
                // PurchaseSlot() - buying the next bank bag slot, which the
                // confirmation dialog's Okay calls. Unbound, so the dialog
                // asked, took the answer and bought nothing. GetBankSlotCost
                // and GetNumBankSlots beside it were both already bound, which
                // is what let the dialog price the slot it could not buy.
                {"PurchaseSlot", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L)) gh->buyBankSlot();
            return 0;
        }},
                {"ContainerIDToInventoryID", lua_ContainerIDToInventoryID},
                {"PutItemInBackpack",       lua_PutItemInBackpack},
                {"GetBagName",              lua_GetBagName},
                {"SetBagPortraitTexture",   lua_SetBagPortraitTexture},
                {"PutItemInBag",            lua_PutItemInBag},
                {"ResetCursor",             lua_ResetCursor},
                {"GetContainerItemID",    lua_GetContainerItemID},
                {"GetContainerFreeSlots", lua_GetContainerFreeSlots},
                {"GetLatestThreeSenders", lua_GetLatestThreeSenders},
                {"PickupEquipmentSetByName", lua_PickupEquipmentSetByName},
                {"GetContainerItemDurability", lua_GetContainerItemDurability},
                {"GetContainerItemGems",  lua_ItemGemsNone},
                {"GetInventoryItemGems",  lua_ItemGemsNone},
                {"GetContainerItemInfo",    lua_GetContainerItemInfo},
                {"GetContainerItemLink",    lua_GetContainerItemLink},
                {"GetContainerNumFreeSlots", lua_GetContainerNumFreeSlots},
                {"GetInventorySlotInfo",    lua_GetInventorySlotInfo},
                {"GetInventoryItemLink",    lua_GetInventoryItemLink},
                {"GetInventoryItemCount",   lua_GetInventoryItemCount},
                // How many rows the buyback tab has. The merchant window walks
                // this to build the tab, and without it the tab was empty even
                // with items sitting in the buyback ring the client tracks.
                {"GetNumBuybackItems", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushnumber(L, gh ? static_cast<lua_Number>(gh->getBuybackItems().size()) : 0);
            return 1;
        }},
                {"GetInventoryItemID",      lua_GetInventoryItemID},
                {"GetInventoryItemTexture", lua_GetInventoryItemTexture},
                {"GetItemLink",          lua_GetItemLink},
                {"GetNumLootItems",     lua_GetNumLootItems},
                {"GetLootSlotInfo",     lua_GetLootSlotInfo},
                {"GetLootSlotLink",     lua_GetLootSlotLink},
                {"LootSlot",            lua_LootSlot},
                // ConfirmLootSlot(slot) - the Accept on "this will bind to
                // you", raised from LOOT_BIND_CONFIRM.
                //
                // Not LootSlot again: LootSlot is what raises the prompt, so
                // answering it that way asks the same question forever. The
                // held request is sent instead, and the slot the dialog passes
                // is not read - only one can be waiting.
                {"ConfirmLootSlot", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L)) gh->confirmPendingLoot();
            return 0;
        }},
                // The answer to "this item will bind to you".
                //
                // Both dialogs - EQUIP_BIND for a drop onto a slot,
                // AUTOEQUIP_BIND for a right-click - call these two, and both
                // pass the slot the event carried. Only one equip can be
                // waiting at a time (FrameXML's dialog is exclusive and this
                // client's is modal), so the held request is enough on its own
                // and the slot is not read back.
                //
                // OnHide calls CancelPendingEquip as well as OnCancel, so
                // cancelling twice has to be harmless - it is: the second
                // clears an already-empty request.
                // ConfirmBindOnUse() - the Accept on "using this will bind
                // it". USE_BIND carries no slot, so the held request is the
                // only record of which item was being used.
                // ReplaceEnchant() - the Yes on "do you want to replace X
                // with Y?". The enchant stays parked until this arrives, so
                // the answer still has the item it was aimed at.
                //
                // TRADE_REPLACE_ENCHANT shares this button. That prompt is the
                // one raised while an enchant is on the trade window, which
                // this client does not put there - so it is never fired, and
                // the shared verb costs nothing.
                // BindEnchant() - the Okay on "enchanting this will bind it".
                // The same held request ReplaceEnchant answers: only one
                // enchant can be waiting, and the two prompts are raised from
                // the same place for the same pending cast.
                {"BindEnchant", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L)) gh->replaceEnchant();
            return 0;
        }},
                {"ReplaceEnchant", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L)) gh->replaceEnchant();
            return 0;
        }},
                // RespondMailLockSendItem(slot, keep) - the answer to "this
                // item can still be handed back; posting it ends that". Keep
                // it attached, or take it off again.
                //
                // MAIL_UNLOCK_SEND_ITEMS either way: the lock is the dialog
                // being up, and it comes down whichever button was pressed.
                {"RespondMailLockSendItem", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int slot = static_cast<int>(luaL_optnumber(L, 1, 0));
            const bool keep = lua_toboolean(L, 2) != 0;
            if (!gh) return 0;
            if (!keep && slot >= 1) gh->detachMailAttachment(slot - 1);
            gh->fireAddonEvent("MAIL_UNLOCK_SEND_ITEMS", {});
            return 0;
        }},
                {"ConfirmBindOnUse", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L)) gh->confirmBindOnUse();
            return 0;
        }},
                {"EquipPendingItem", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L)) gh->equipPendingItem();
            return 0;
        }},
                {"CancelPendingEquip", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L)) gh->cancelPendingEquip();
            return 0;
        }},
                {"SplitContainerItem",  lua_SplitContainerItem},
                {"BankButtonIDToInvSlotID", lua_BankButtonIDToInvSlotID},
                {"CloseBankFrame",      lua_CloseBankFrame},
                {"GetInboxItem",        lua_GetInboxItem},
                {"GetInboxItemLink",    lua_GetInboxItemLink},
                {"TakeInboxItem",       lua_TakeInboxItem},
                {"TakeInboxMoney",      lua_TakeInboxMoney},
                {"DeleteInboxItem",     lua_DeleteInboxItem},
                {"InboxItemCanDelete",  lua_InboxItemCanDelete},
                {"AutoLootMailItem",    lua_AutoLootMailItem},
                {"GetSendMailItem",     lua_GetSendMailItem},
                {"CheckInbox",          lua_CheckInbox},
                {"ReturnInboxItem",      lua_ReturnInboxItem},
                {"GetCoinIcon",          lua_GetCoinIcon},
                {"CloseMail",           lua_CloseMail},
                {"SendMail",            lua_SendMail},
                {"SetSendMailMoney",    lua_SetSendMailMoney},
                {"SetSendMailCOD",      lua_SetSendMailCOD},
                {"AddSendMailMoney",    lua_AddSendMailMoney},
                {"AddSendMailCOD",      lua_AddSendMailCOD},
                {"GetSendMailMoney",    lua_GetSendMailMoney},
                {"GetSendMailCOD",      lua_GetSendMailCOD},
                {"UseInventoryItem",    lua_UseInventoryItem},
                {"GetInventoryItemDurability", lua_GetInventoryItemDurability},
                {"UseItemByName",       lua_UseItemByName},
                {"EquipItemByName",     lua_EquipItemByName},
                {"IsEquippableItem",    lua_IsEquippableItem},
                {"IsEquippedItem",      lua_IsEquippedItem},
                {"GetInventoryItemsForSlot", lua_GetInventoryItemsForSlot},
                {"GetNumEquipmentSets", lua_GetNumEquipmentSets},
                {"GetEquipmentSetInfo", lua_GetEquipmentSetInfo},
                {"GetEquipmentSetInfoByName", lua_GetEquipmentSetInfoByName},
                {"GetEquipmentSetItemIDs", lua_GetEquipmentSetItemIDs},
                {"SaveEquipmentSet",    lua_SaveEquipmentSet},
                {"DeleteEquipmentSet",  lua_DeleteEquipmentSet},
                {"UseEquipmentSet",     lua_UseEquipmentSet},
                {"EquipmentSetContainsLockedItems", lua_EquipmentSetContainsLockedItems},
                {"EquipmentManagerIgnoreSlotForSave",   lua_EquipmentManagerIgnoreSlotForSave},
                {"EquipmentManagerUnignoreSlotForSave", lua_EquipmentManagerUnignoreSlotForSave},
                {"EquipmentManagerClearIgnoredSlotsForSave", lua_EquipmentManagerClearIgnoredSlotsForSave},
                {"LootSlotIsCoin",      lua_LootSlotIsCoin},
                {"LootSlotIsItem",      lua_LootSlotIsItem},
                {"IsFishingLoot",       lua_IsFishingLoot},
                {"CloseLoot",           lua_CloseLoot},
                {"GiveMasterLoot",      lua_GiveMasterLoot},
                {"GetLootRollItemInfo", lua_GetLootRollItemInfo},
                {"GetLootRollItemLink", lua_GetLootRollItemLink},
                {"GetLootRollTimeLeft", lua_GetLootRollTimeLeft},
                {"RollOnLoot",          lua_RollOnLoot},
                {"ConfirmLootRoll",     lua_ConfirmLootRoll},
                {"GetLootMethod",       lua_GetLootMethod},
                {"GetMasterLootCandidate", lua_GetMasterLootCandidate},
                {"GetLootThreshold",    lua_GetLootThreshold},
                {"GetTabardCreationCost", lua_GetTabardCreationCost},
                {"GetSendMailPrice",    lua_GetSendMailPrice},
                {"BuyMerchantItem", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            int index = static_cast<int>(luaL_checknumber(L, 1));
            int count = static_cast<int>(luaL_optnumber(L, 2, 1));
            if (!gh || index < 1) return 0;
            const auto& items = gh->getVendorItems().items;
            if (index > static_cast<int>(items.size())) return 0;
            const auto& vi = items[index - 1];
            gh->buyItem(gh->getVendorGuid(), vi.itemId, vi.slot, count);
            return 0;
        }},
                {"SellContainerItem", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            int bag = static_cast<int>(luaL_checknumber(L, 1));
            int slot = static_cast<int>(luaL_checknumber(L, 2));
            if (!gh) return 0;
            if (bag == 0) gh->sellItemBySlot(slot - 1);
            else if (bag >= 1 && bag <= 4) gh->sellItemInBag(bag - 1, slot - 1);
            return 0;
        }},
                {"RepairAllItems", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (gh && gh->getVendorItems().canRepair) {
                bool useGuildBank = lua_toboolean(L, 1) != 0;
                gh->repairAll(gh->getVendorGuid(), useGuildBank);
            }
            return 0;
        }},
                {"UnequipItemSlot", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            int slot = static_cast<int>(luaL_checknumber(L, 1));
            if (gh && slot >= 1 && slot <= 19)
                gh->unequipToBackpack(static_cast<game::EquipSlot>(slot - 1));
            return 0;
        }},
                {"AcceptTrade", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (gh) gh->acceptTrade();
            return 0;
        }},
                {"CancelTrade", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (gh && gh->isTradeOpen()) gh->cancelTrade();
            return 0;
        }},
                {"InitiateTrade", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* uid = luaL_checkstring(L, 1);
            if (gh) {
                uint64_t guid = resolveUnitGuid(gh, std::string(uid));
                if (guid != 0) gh->initiateTrade(guid);
            }
            return 0;
        }},
                // ---- Guild bank -----------------------------------------
                //
                // The client opens it, queries a tab, moves items and money in
                // and out, and holds what came back. None of it reached the
                // interface.
                {"GetGuildBankMoney", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushnumber(L, gh ? static_cast<double>(
                gh->getGuildBankData().money) : 0.0);
            return 1;
        }},
                {"GetGuildBankWithdrawMoney", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            // -1 is the server saying "no limit", and the panel reads that as
            // a number to compare against - a large one keeps the comparison
            // true without pretending to a figure.
            const int32_t w = gh ? gh->getGuildBankData().withdrawAmount : 0;
            lua_pushnumber(L, w < 0 ? 100000000.0 : static_cast<double>(w));
            return 1;
        }},
                {"CanWithdrawGuildBankMoney", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int32_t w = gh ? gh->getGuildBankData().withdrawAmount : 0;
            lua_pushboolean(L, (w != 0) ? 1 : 0);
            return 1;
        }},
                {"GetCurrentGuildBankTab", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushnumber(L, gh ? (gh->getGuildBankActiveTab() + 1) : 1);
            return 1;
        }},
                {"SetCurrentGuildBankTab", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int tab = static_cast<int>(luaL_optnumber(L, 1, 1));
            if (gh && tab >= 1) {
                gh->setGuildBankActiveTab(static_cast<uint8_t>(tab - 1));
            }
            return 0;
        }},
                {"QueryGuildBankTab", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int tab = static_cast<int>(luaL_optnumber(L, 1, 1));
            if (gh && tab >= 1) gh->queryGuildBankTab(static_cast<uint8_t>(tab - 1));
            return 0;
        }},
                // How many tabs the guild has bought. blizzard_guildbankui does
                // `elseif ( tab > GetNumGuildBankTabs() )` to decide whether to
                // offer the buy screen, and comparing a number against nil
                // raises - so the window died on any tab past the last one.
                //
                // The list it counts is the same one GetGuildBankTabInfo
                // indexes, so the two cannot disagree.
                {"GetNumGuildBankTabs", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushnumber(L, gh ? static_cast<lua_Number>(gh->getGuildBankData().tabs.size()) : 0);
            return 1;
        }},
                // GetGuildBankTabInfo(tab) → name, icon, viewable, canDeposit,
                //                            numWithdrawals, remaining
                {"GetGuildBankTabInfo", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int tab = static_cast<int>(luaL_optnumber(L, 1, 0));
            const auto* tabs = gh ? &gh->getGuildBankData().tabs : nullptr;
            // A tab the guild has not bought answers as an empty tab rather
            // than as one nil, and the difference is a raise.
            //
            // Every caller unpacks all six and the panel walks 1..
            // MAX_GUILDBANK_TABS whatever the guild owns, so the tail of that
            // loop always lands here. With a single nil the sixth value is nil
            // and GuildBankFrame_Update does `if remainingWithdrawals > 0`,
            // which is a comparison against nil - it takes the whole update
            // down part way, and a guild with no tabs at all never gets past
            // it.
            //
            // The numbers are zeros and the strings empty, which is what the
            // panel reads as "locked, nothing withdrawable"; it substitutes its
            // own "Tab N" for the empty name a line later.
            if (!tabs || tab < 1 || tab > static_cast<int>(tabs->size())) {
                lua_pushstring(L, "");
                lua_pushstring(L, "");
                lua_pushboolean(L, 0);   // not viewable
                lua_pushboolean(L, 0);   // cannot deposit
                lua_pushnumber(L, 0);    // withdrawals allowed
                lua_pushnumber(L, 0);    // withdrawals remaining
                return 6;
            }
            const auto& t = (*tabs)[tab - 1];
            lua_pushstring(L, t.tabName.c_str());
            lua_pushstring(L, t.tabIcon.c_str());
            lua_pushboolean(L, 1);   // viewable: it was sent, so it is
            lua_pushboolean(L, 1);   // canDeposit
            lua_pushnumber(L, 0);
            lua_pushnumber(L, 0);
            return 6;
        }},
                // GetGuildBankItemInfo(tab, slot) → texture, count, locked
                {"GetGuildBankItemInfo", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int tab  = static_cast<int>(luaL_optnumber(L, 1, 0));
            const int slot = static_cast<int>(luaL_optnumber(L, 2, 0));
            if (!gh) return luaReturnNil(L);
            const auto* found = guildBankItemAt(gh, tab, slot);
            if (!found) return luaReturnNil(L);   // empty slot
            const auto& it = *found;
            gh->ensureItemInfo(it.itemEntry);
            const auto* info = gh->getItemInfo(it.itemEntry);
            const std::string icon =
                info ? gh->getItemIconPath(info->displayInfoId) : std::string();
            lua_pushstring(L, icon.empty()
                ? "Interface\\Icons\\INV_Misc_QuestionMark" : icon.c_str());
            lua_pushnumber(L, it.stackCount);
            lua_pushboolean(L, 0);   // locked
            return 3;
        }},
                // GetGuildBankItemLink(tab, slot) → hyperlink
                {"GetGuildBankItemLink", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int tab  = static_cast<int>(luaL_optnumber(L, 1, 0));
            const int slot = static_cast<int>(luaL_optnumber(L, 2, 0));
            if (!gh) return luaReturnNil(L);
            const auto* found = guildBankItemAt(gh, tab, slot);
            if (!found) return luaReturnNil(L);   // empty slot
            const auto& it = *found;
            gh->ensureItemInfo(it.itemEntry);
            const auto* info = gh->getItemInfo(it.itemEntry);
            if (!info) return luaReturnNil(L);
            // The same shape GetContainerItemLink builds, so a link from
            // the guild bank behaves like one from a bag everywhere it is
            // handed on to.
            const std::string link = game::itemChatLink(
                it.itemEntry, info->quality, info->name, it.enchantId,
                static_cast<int32_t>(it.randomPropertyId));
            lua_pushstring(L, link.c_str());
            return 1;
        }},
                // Moving an item within the bank needs a cursor that can hold
                // a guild bank slot, which this client does not model - the
                // withdraw and deposit packets move an item straight to or
                // from a bag. AutoStoreGuildBankItem does that and works;
                // these two say nothing rather than half-moving something.
                {"CloseGuildBankFrame", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L)) gh->closeGuildBank();
            return 0;
        }},
                {"DepositGuildBankMoney", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (gh) gh->depositGuildBankMoney(
                static_cast<uint32_t>(luaL_optnumber(L, 1, 0)));
            return 0;
        }},
                {"WithdrawGuildBankMoney", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (gh) gh->withdrawGuildBankMoney(
                static_cast<uint32_t>(luaL_optnumber(L, 1, 0)));
            return 0;
        }},
                {"AutoStoreGuildBankItem", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int tab  = static_cast<int>(luaL_optnumber(L, 1, 1));
            const int slot = static_cast<int>(luaL_optnumber(L, 2, 1));
            // Destination 0,0 lets the server pick the first free bag slot,
            // which is what "auto store" means.
            if (gh) gh->guildBankWithdrawItem(static_cast<uint8_t>(tab - 1),
                                              static_cast<uint8_t>(slot - 1), 0, 0);
            return 0;
        }},
                // Rank *is* in what this client parses - the roster carries a
                // rankIndex per member and the chat handler has been reading
                // the player's own to decide whether to show officer chat. The
                // comment that used to sit here said otherwise, and answering
                // no meant a guild master was offered none of what is theirs.
                // Rank zero is the guild master; an unknown rank is not.
                {"IsGuildLeader", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushboolean(L, gh && gh->getPlayerGuildRankIndex() == 0 ? 1 : 0);
            return 1;
        }},
                // The transaction log, the tab text and the tabard are not in
                // what this client parses. Each answers empty rather than
                // inventing a history nobody made.
                // The log tab asks for a tab's history and then walks it.
                // This accepted the ask and sent nothing, so the walk below
                // always found none - an empty page over a log the server
                // keeps. Tab seven in FrameXML's numbering is the money log.
                {"QueryGuildBankLog", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int tab = static_cast<int>(luaL_optnumber(L, 1, 1));
            if (gh && tab >= 1) gh->requestGuildBankLog(static_cast<uint8_t>(tab - 1));
            return 0;
        }},
                // Ask for a tab's info text. The panel calls this when a tab is
                // opened and reads the answer with GetGuildBankText below.
                {"QueryGuildBankText",  [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int tab = static_cast<int>(luaL_optnumber(L, 1, 0));
            if (gh && tab >= 1) gh->queryGuildBankText(static_cast<uint8_t>(tab - 1));
            return 0;
        }},
                // What the query above brought back, or nil for a tab nothing
                // has answered for yet. Still nil rather than "": the panel
                // does `if ( text )` and its else branch clears the box, so an
                // empty string would claim to know the tab has none.
                {"GetGuildBankText",    [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int tab = static_cast<int>(luaL_optnumber(L, 1, 0));
            if (gh && tab >= 1) {
                const std::string& text = gh->getGuildBankTabText(static_cast<uint8_t>(tab - 1));
                if (!text.empty()) { lua_pushstring(L, text.c_str()); return 1; }
            }
            lua_pushnil(L);
            return 1;
        }},
                // SetGuildBankText(tab, text) - the info panel on a guild
                // bank tab. The opcode existed and nothing built it, so the
                // edit box saved nothing. FrameXML counts tabs from one and
                // the wire counts from zero, the same offset
                // GetCurrentGuildBankTab already applies in reverse.
                {"SetGuildBankText", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int tab = static_cast<int>(luaL_optnumber(L, 1, 0));
            const char* text = luaL_optstring(L, 2, "");
            if (gh && tab >= 1)
                gh->setGuildBankTabText(static_cast<uint8_t>(tab - 1), text ? text : "");
            return 0;
        }},
                {"GetNumGuildBankTransactions", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int tab = static_cast<int>(luaL_optnumber(L, 1, 1));
            lua_pushnumber(L, (gh && tab >= 1)
                ? static_cast<lua_Number>(gh->getGuildBankLog(static_cast<uint8_t>(tab - 1)).size())
                : 0);
            return 1;
        }},
                {"GetNumGuildBankMoneyTransactions", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushnumber(L, gh
                ? static_cast<lua_Number>(gh->getGuildBankLog(kGuildBankMoneyTab).size())
                : 0);
            return 1;
        }},
                // type, name, itemLink, count, tab1, tab2, year, month, day, hour
                {"GetGuildBankTransaction", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int tab = static_cast<int>(luaL_optnumber(L, 1, 1));
            const int index = static_cast<int>(luaL_optnumber(L, 2, 0));
            if (!gh || tab < 1 || index < 1) return 0;
            const auto& log = gh->getGuildBankLog(static_cast<uint8_t>(tab - 1));
            if (index > static_cast<int>(log.size())) return 0;
            const auto& e = log[static_cast<size_t>(index) - 1];

            lua_pushstring(L, bankLogTypeName(e.type));
            lua_pushstring(L, gh->lookupName(e.playerGuid).c_str());
            pushItemLinkOrNil(L, gh, e.itemId);
            lua_pushnumber(L, e.count);
            lua_pushnumber(L, tab);                  // the tab it happened on
            lua_pushnumber(L, e.otherTab + 1);       // and the one it moved to
            pushTimeAgo(L, e.secondsAgo);
            return 10;
        }},
                // type, name, amount, year, month, day, hour
                {"GetGuildBankMoneyTransaction", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int index = static_cast<int>(luaL_optnumber(L, 1, 0));
            if (!gh || index < 1) return 0;
            const auto& log = gh->getGuildBankLog(kGuildBankMoneyTab);
            if (index > static_cast<int>(log.size())) return 0;
            const auto& e = log[static_cast<size_t>(index) - 1];

            lua_pushstring(L, bankLogTypeName(e.type));
            lua_pushstring(L, gh->lookupName(e.playerGuid).c_str());
            lua_pushnumber(L, e.count);              // copper
            pushTimeAgo(L, e.secondsAgo);
            return 7;
        }},
                // The six textures a guild tabard is drawn from, in the order
                // the guild bank unpacks them: background upper and lower,
                // emblem upper and lower, border upper and lower.
                //
                // This answered nothing, and the data was in hand the whole
                // time - SMSG_GUILD_QUERY_RESPONSE carries the emblem style
                // and colour, the border style and colour and the background
                // colour, and handleGuildQueryResponse keeps the whole struct.
                // A grep for the field names does not see that, because it is
                // assigned wholesale as `guildQueryData_ = data`.
                //
                // The names are built rather than looked up: the install has
                // 6118 of these and they are named by index -
                // background_<colour>, emblem_<style>_<colour>,
                // border_<style>_<colour>, each in an upper and a lower half.
                // Ranges measured from the files present rather than guessed,
                // and the guess was wrong both ways: 51 backgrounds, 170 emblem
                // styles - not 100 - of 17 colours each, and 10 border styles
                // of which the first six have 17 colours and the last four have
                // only 4.
                //
                // Out of range answers nothing rather than naming a file that
                // is not there, which is what the server sends for a guild with
                // no emblem chosen. The guild bank guards for that and falls
                // back to its own default background - so a wrong name would be
                // worse than no name, since a non-nil one stops that fallback.
                {"GetGuildTabardFileNames", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) return 0;
            const auto& g = gh->getGuildQueryData();
            if (g.guildId == 0) return 0;
            const uint32_t kBorderColors = (g.borderStyle <= 5) ? 16u : 3u;
            if (g.backgroundColor > 50 || g.emblemStyle > 169 ||
                g.emblemColor > 16 || g.borderStyle > 9 ||
                g.borderColor > kBorderColors) {
                return 0;
            }
            auto push = [&](const char* kind, uint32_t a, int b, const char* half) {
                char buf[128];
                if (b < 0) {
                    std::snprintf(buf, sizeof(buf),
                                  "Textures\\GuildEmblems\\%s_%02u_%s_U", kind, a, half);
                } else {
                    std::snprintf(buf, sizeof(buf),
                                  "Textures\\GuildEmblems\\%s_%02u_%02d_%s_U",
                                  kind, a, b, half);
                }
                lua_pushstring(L, buf);
            };
            push("Background", g.backgroundColor, -1, "TU");
            push("Background", g.backgroundColor, -1, "TL");
            push("Emblem", g.emblemStyle, static_cast<int>(g.emblemColor), "TU");
            push("Emblem", g.emblemStyle, static_cast<int>(g.emblemColor), "TL");
            push("Border", g.borderStyle, static_cast<int>(g.borderColor), "TU");
            push("Border", g.borderStyle, static_cast<int>(g.borderColor), "TL");
            return 6;
        }},
                // The guild master, which is what the real client answers and
                // is now a question this can put - rank zero in the roster. It
                // used to say no on the grounds that nothing could send the
                // change; that was true of the builder and not of the opcode,
                // which has been in all three expansion maps.
                //
                // AzerothCore does not gate the rename by rank at all, so this
                // is the client being stricter than the server, as retail is.
                {"CanEditGuildTabInfo", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            lua_pushboolean(L, gh && gh->getPlayerGuildRankIndex() == 0 ? 1 : 0);
            return 1;
        }},
                // SetGuildBankTabInfo(tab, name, icon) - the rename popup's
                // Okay. The tab is FrameXML's, counted from one.
                {"SetGuildBankTabInfo", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int tab = static_cast<int>(luaL_optnumber(L, 1, 0));
            const char* name = luaL_optstring(L, 2, "");
            const char* icon = luaL_optstring(L, 3, "");
            if (gh && tab >= 1)
                gh->setGuildBankTabInfo(static_cast<uint8_t>(tab - 1), name, icon);
            return 0;
        }},
                // What the next tab costs, in copper, or nil once all six are
                // bought - and nil is the load-bearing part: the panel does
                // `if ( not tabCost )` to decide the guild has them all, and
                // zero is true in Lua, so answering zero kept the buy screen up
                // for a guild with nothing left to buy and priced it at nothing.
                //
                // BuyGuildBankTab does send, so this was not a dead number on a
                // dead screen: it offered a real purchase at a made-up price.
                // The six are the client's own constants in WoW rather than
                // anything the server sends, and AzerothCore's defaults are the
                // same figures - 100g, 250g, 500g, 1000g, 2500g, 5000g - though
                // Guild.BankTabCost0-5 can be configured away from them.
                {"GetGuildBankTabCost", [](lua_State* L) -> int {
            static constexpr uint32_t kTabCost[6] = {
                1000000u, 2500000u, 5000000u, 10000000u, 25000000u, 50000000u
            };
            auto* gh = getGameHandler(L);
            const size_t owned = gh ? gh->getGuildBankData().tabs.size() : 0;
            if (owned >= 6) { lua_pushnil(L); return 1; }
            lua_pushnumber(L, static_cast<lua_Number>(kTabCost[owned]));
            return 1;
        }},
                // ---- Auction house, the acting half ---------------------
                //
                // The listing half was already here; these are the calls that
                // search, bid, sell and cancel, all of which the client can
                // send and none of which the interface could reach.
                {"CanSendAuctionQuery", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            // Two returns: whether a search may be sent, and whether a
            // getAll sweep may be. The second is always no - it asks the
            // server for every auction at once and is rate-limited to once
            // every fifteen minutes even where it is allowed.
            const bool open = gh && gh->isAuctionHouseOpen();
            lua_pushboolean(L, open && gh->getAuctionSearchDelay() <= 0.0f);
            lua_pushboolean(L, 0);
            return 2;
        }},
                // QueryAuctionItems(name, minLevel, maxLevel, invType, class,
                //                   subclass, page, isUsable, quality)
                {"QueryAuctionItems", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) return 0;
            const char* name = luaL_optstring(L, 1, "");
            // Every argument here is a widget's own output, handed over
            // untouched by AuctionFrameBrowse_SearchHelper - so the level
            // bounds arrive as the text of an edit box. An empty box gives ""
            // rather than nil, which luaL_optnumber does not treat as absent
            // but as a string that will not convert, and raises on. The level
            // boxes start empty, so the commonest search of all - type a name,
            // press Search - died here before a single byte went out.
            const uint8_t lo  = static_cast<uint8_t>(luaOptNumberText(L, 2, 0));
            const uint8_t hi  = static_cast<uint8_t>(luaOptNumberText(L, 3, 0));
            // The three filters arrive as positions in the lists above, not as
            // the ids the wire wants - and nil, for "not filtered", arrives as
            // zero. Zero is a real item class, so this used to ask the server
            // for Consumables whenever nobody had picked a category: a plain
            // search by name found nothing but food. Row zero of each shared
            // list is "All", so the lookup turns both cases into the right
            // answer at once.
            const int invIdx = static_cast<int>(luaOptNumberText(L, 4, 0));
            const int clsIdx = static_cast<int>(luaOptNumberText(L, 5, 0));
            const int subIdx = static_cast<int>(luaOptNumberText(L, 6, 0));
            const uint32_t cls = (clsIdx > 0 && clsIdx < game::kNumAuctionClasses)
                ? game::kAuctionClasses[clsIdx].classId : game::kAuctionAny;
            // The slot arrives as a position in the list GetAuctionInvTypes
            // offered for this class, which is a subset - so it has to be read
            // back through the same subset rather than straight off the table.
            const uint32_t inv = game::auctionSlotIdAt(cls, subIdx, invIdx);
            uint32_t sub = game::kAuctionAny;
            if (cls != game::kAuctionAny && subIdx > 0) {
                int count = 0;
                const auto* subs = game::auctionSubsFor(cls, count);
                if (subs && subIdx < count) sub = subs[subIdx].subId;
            }
            const uint32_t page = static_cast<uint32_t>(luaOptNumberText(L, 7, 0));
            const uint8_t usable = lua_toboolean(L, 8) ? 1 : 0;
            // The rarity dropdown's "All" is -1, not nil, and casting a
            // negative double straight to uint32_t is undefined - it happens to
            // land on the 0xFFFFFFFF the server reads as "any quality" on the
            // machines this has run on, which is not the same as meaning to.
            const int qualityIdx = static_cast<int>(luaOptNumberText(L, 9, -1));
            const uint32_t quality = qualityIdx < 0
                ? game::kAuctionAny : static_cast<uint32_t>(qualityIdx);
            // The page is a page; the wire wants the row it starts at.
            //
            // The ordering travels with the query rather than being applied to
            // what comes back. That is the browse tab's own design: clicking a
            // column header calls AuctionFrameBrowse_Search, not
            // SortAuctionApplySort, so nothing sorts the browse list locally
            // and a query that leaves the sort block empty leaves the headers
            // inert.
            gh->auctionSearch(name, lo, hi, quality, cls, sub, inv, usable,
                              page * 50, wireAuctionSort("list"));
            return 0;
        }},
                {"GetOwnerAuctionItems", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L)) gh->auctionListOwnerItems(0);
            return 0;
        }},
                {"GetBidderAuctionItems", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L)) gh->auctionListBidderItems(0);
            return 0;
        }},
                // PlaceAuctionBid(list, index, bid) - a bid equal to the
                // buyout is a buyout, which is how the panel asks for one.
                {"PlaceAuctionBid", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* list = luaL_optstring(L, 1, "list");
            const int index = static_cast<int>(luaL_optnumber(L, 2, 0));
            const uint32_t bid = static_cast<uint32_t>(luaL_optnumber(L, 3, 0));
            if (!gh) return 0;
            const auto& res = auctionListFor(gh, list);
            if (index < 1 || index > static_cast<int>(res.auctions.size())) return 0;
            const auto& a = res.auctions[index - 1];
            if (a.buyoutPrice != 0 && bid >= a.buyoutPrice) {
                gh->auctionBuyout(a.auctionId, a.buyoutPrice);
            } else {
                gh->auctionPlaceBid(a.auctionId, bid);
            }
            return 0;
        }},
                {"CancelAuction", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int index = static_cast<int>(luaL_optnumber(L, 1, 0));
            if (!gh) return 0;
            const auto& res = gh->getAuctionOwnerResults();
            if (index < 1 || index > static_cast<int>(res.auctions.size())) return 0;
            gh->auctionCancelItem(res.auctions[index - 1].auctionId);
            return 0;
        }},
                {"CanCancelAuction", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const int index = static_cast<int>(luaL_optnumber(L, 1, 0));
            const auto* res = gh ? &gh->getAuctionOwnerResults() : nullptr;
            // Only one that has not been bid on: cancelling after a bid costs
            // a fee this client does not model, and the server refuses anyway.
            const bool ok = res && index >= 1 &&
                            index <= static_cast<int>(res->auctions.size()) &&
                            res->auctions[index - 1].currentBid == 0;
            lua_pushboolean(L, ok ? 1 : 0);
            return 1;
        }},
                // StartAuction(minBid, buyout, duration, stackSize, numStacks)
                {"StartAuction", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const uint32_t bid = static_cast<uint32_t>(luaL_optnumber(L, 1, 0));
            const uint32_t buy = static_cast<uint32_t>(luaL_optnumber(L, 2, 0));
            // The dropdown's 1, 2 and 3 mean twelve, twenty-four and forty-eight
            // hours; the wire field is minutes. Sent as they came, they asked
            // for one, two or three minutes, and AzerothCore accepts only 1, 2
            // or 4 times MIN_AUCTION_TIME and returns without answering - so
            // posting an auction from FrameXML did nothing at all, silently.
            const uint32_t dur = game::auctionDurationMinutes(
                static_cast<uint32_t>(luaL_optnumber(L, 3, 1)));
            uint64_t guid = 0;
            const auto* item = auctionSellItemSlot(gh, &guid);
            if (!item || guid == 0) return 0;
            // The fourth argument is the size of one stack; the fifth is how
            // many such stacks to post, which this sends one of - the request
            // builder writes a single item and posting several would be a
            // several-item list.
            const uint32_t stack = static_cast<uint32_t>(
                luaL_optnumber(L, 4, item->item.stackCount ? item->item.stackCount : 1));
            gh->auctionSellItemByGuid(guid, stack, bid, buy, dur);
            auctionSellSlot() = AuctionSellSlot{};
            return 0;
        }},
                // The sell slot: the item the player dropped on it, held here
                // because it is the panel's own state until StartAuction sends
                // it.
                {"ClickAuctionSellItemButton", [](lua_State* L) -> int {
            // The slot had state and StartAuction read it, but nothing ever
            // set it: this was a no-op, so no item could be put up for auction
            // at all. What it needed was a drop target, and the cursor bridge
            // is one - the item the player is carrying is exactly what a click
            // on this button is offering.
            //
            uint8_t bag = 0, slot = 0;
            if (!wowee::ui::frameXmlCursorWireSlot(bag, slot)) {
                // Nothing carried: a click takes the item back out.
                auctionSellSlot() = AuctionSellSlot{};
                return 0;
            }
            auto* gh = getGameHandler(L);
            AuctionSellSlot candidate{.held = true, .bag = bag, .slot = slot};
            AuctionSellSlot previous = auctionSellSlot();
            auctionSellSlot() = candidate;
            // Only if it names something. Equipment and the bank produce wire
            // pairs too, and neither can be auctioned from here.
            if (!auctionSellItemSlot(gh)) {
                auctionSellSlot() = previous;
                return 0;
            }
            wowee::ui::frameXmlPutCursorDown();
            return 0;
        }},
                {"CancelSell", [](lua_State* L) -> int {
            auctionSellSlot() = AuctionSellSlot{};
            (void)L;
            return 0;
        }},
                {"GetAuctionSellItemInfo", [](lua_State* L) -> int {
            // name, texture, count, quality, canUse, price, pricePerUnit,
            // stackCount, totalCount - what the sell tab draws in its slot and
            // what its deposit and stack controls are figured from.
            //
            // The last three were missing, and the tab does `if totalCount > 1`
            // the moment an item is dropped in, so it raised on a nil rather
            // than showing a slot without stack controls. stackCount is the
            // item's maximum stack and totalCount is how many the player holds:
            // UpdateMaximumButtons caps the stack size at min(totalCount,
            // stackCount) and the number of stacks at totalCount / stackSize.
            auto* gh = getGameHandler(L);
            const auto* held = auctionSellItemSlot(gh);
            if (!held) { auctionSellSlot() = AuctionSellSlot{}; return luaReturnNil(L); }
            const auto& s = *held;
            const auto* info = gh->getItemInfo(s.item.itemId);
            const uint32_t inStack = s.item.stackCount ? s.item.stackCount : 1;
            // price is the vendor value of the whole stack and pricePerUnit of
            // one of them - the tab offers a starting bid of one or the other
            // depending on which way the price dropdown is set, so sending the
            // per-unit figure as both had it suggest a stack for the price of
            // a single item.
            const uint32_t unitPrice = (info && info->valid) ? info->sellPrice : 0;
            lua_pushstring(L, (info && info->valid && !info->name.empty())
                                  ? info->name.c_str() : s.item.name.c_str());
            lua_pushstring(L, gh->getItemIconPath(s.item.displayInfoId).c_str());
            lua_pushnumber(L, inStack);
            lua_pushnumber(L, info && info->valid ? info->quality
                                  : static_cast<uint32_t>(s.item.quality));
            lua_pushboolean(L, 1);
            lua_pushnumber(L, static_cast<lua_Number>(unitPrice) * inStack);
            lua_pushnumber(L, unitPrice);
            lua_pushnumber(L, (info && info->valid && info->maxStack > 0)
                                  ? static_cast<lua_Number>(info->maxStack) : 1);
            lua_pushnumber(L, countItemInBags(gh, s.item.itemId));
            return 9;
        }},
                {"CloseAuctionHouse", [](lua_State* L) -> int {
            if (auto* gh = getGameHandler(L)) gh->closeAuctionHouse();
            return 0;
        }},
                // SetAuctionsTabShowing(showing) - a *boolean*, not a tab index.
                //
                // It is the last statement in each branch of
                // AuctionFrameTab_OnClick, and it was reading its argument with
                // optnumber: `SetAuctionsTabShowing(false)` raised "number
                // expected, got boolean" on every click of the Browse and Bids
                // tabs, and the interface's own handler ended there.
                //
                // The name says what it means. It is not "which tab" but
                // "is the Auctions tab the one on screen" - the interface calls
                // it with false from Browse and from Bids, and true only from
                // Auctions. So false cannot say which of the other two it is,
                // and nothing here may pretend otherwise: the one reader is
                // this client's own auction window, which is not drawn at all
                // while the interface owns the panel.
                //
                // A number is still accepted, because an addon written against
                // the older shape of this binding would otherwise start raising
                // where it used to work.
                {"SetAuctionsTabShowing", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) return 0;
            if (lua_isboolean(L, 1)) {
                gh->setAuctionActiveTab(lua_toboolean(L, 1) ? 2 : 0);
            } else if (lua_isnumber(L, 1)) {
                gh->setAuctionActiveTab(static_cast<int>(lua_tonumber(L, 1)));
            } else if (lua_isnoneornil(L, 1)) {
                gh->setAuctionActiveTab(0);
            }
            return 0;
        }},
                {"SetSelectedAuctionItem", [](lua_State* L) -> int {
            auctionSelection() = static_cast<int>(luaL_optnumber(L, 2, 0));
            return 0;
        }},
                {"GetSelectedAuctionItem", [](lua_State* L) -> int {
            lua_pushnumber(L, auctionSelection());
            return 1;
        }},
                // CalculateAuctionDeposit(minutes [, count]) → copper
                //
                // AuctionHouseMgr::GetAuctionDeposit, which is what the server
                // will actually charge:
                //
                //   deposit = depositPercent * 3 / 100 * sellPrice * count
                //             * (minutes / 720)
                //
                // floored at a hundred copper. depositPercent comes from
                // AuctionHouse.dbc - five for a faction house, twenty-five for
                // the neutral one - and the twelve-hour block is the unit, so
                // the three durations multiply by one, two and four.
                //
                // It reported zero before, which was honest while the sell slot
                // could not hold anything. It can now.
                {"CalculateAuctionDeposit", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            // The same duration the dropdown hands StartAuction, and in the
            // same 1..3 shape: read as minutes it made `blocks` zero below, so
            // every deposit was the one-silver floor whatever the item or the
            // length.
            const double minutes = static_cast<double>(game::auctionDurationMinutes(
                static_cast<uint32_t>(luaL_optnumber(L, 1, 1))));
            const auto* held = auctionSellItemSlot(gh);
            if (!held || minutes <= 0) {
                lua_pushnumber(L, 0);
                return 1;
            }
            const auto& s = *held;
            const auto* info = gh->getItemInfo(s.item.itemId);
            const double sellPrice = (info && info->valid) ? info->sellPrice : 0.0;
            constexpr double kMinimumDeposit = 100.0;
            if (sellPrice <= 0.0) {
                lua_pushnumber(L, kMinimumDeposit);
                return 1;
            }
            const double count = luaL_optnumber(L, 2, s.item.stackCount ? s.item.stackCount : 1);
            // Faction houses take five percent, the neutral one twenty-five.
            // Which one this is is not tracked, so the cheaper is assumed -
            // understating a deposit is the kinder way to be wrong, and it is
            // right at every auctioneer but Booty Bay's.
            constexpr double kDepositPercent = 5.0;
            const double blocks = std::floor(minutes / 720.0);
            const double deposit = (kDepositPercent * 3.0 / 100.0) * sellPrice
                                   * count * blocks;
            lua_pushnumber(L, deposit < kMinimumDeposit ? kMinimumDeposit : deposit);
            return 1;
        }},
                // Sorting is the panel's own, applied to what the server sent.
                {"SortAuctionSetSort", [](lua_State* L) -> int {
            const std::string which = luaL_optstring(L, 1, "list");
            const char* column = luaL_optstring(L, 2, "");
            if (!column || !*column) return 0;
            auctionSortState()[which].push_back({column, lua_toboolean(L, 3) != 0});
            return 0;
        }},
                {"SortAuctionClearSort", [](lua_State* L) -> int {
            auctionSortState()[luaL_optstring(L, 1, "list")].clear();
            return 0;
        }},
                // GetAuctionSort(table, index) → column, reverse.
                //
                // Index one is the primary, and the primary is the key set
                // *last* - FrameXML pushes them least significant first.
                {"GetAuctionSort", [](lua_State* L) -> int {
            const auto& keys = auctionSortState()[luaL_optstring(L, 1, "list")];
            const int index = static_cast<int>(luaL_optnumber(L, 2, 1));
            if (index < 1 || index > static_cast<int>(keys.size())) return 0;
            const auto& k = keys[keys.size() - static_cast<size_t>(index)];
            lua_pushstring(L, k.column.c_str());
            lua_pushboolean(L, k.reverse ? 1 : 0);
            return 2;
        }},
                // Applied least significant first, each pass stable, which is
                // how a multi-column sort is built out of single-column ones.
                {"SortAuctionApplySort", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const std::string which = luaL_optstring(L, 1, "list");
            auto* list = auctionListForSort(gh, which);
            if (!list) return 0;
            for (const auto& key : auctionSortState()[which]) {
                std::stable_sort(list->auctions.begin(), list->auctions.end(),
                                 [&](const game::AuctionEntry& a,
                                     const game::AuctionEntry& b) {
                    return key.reverse ? auctionLess(gh, key.column, b, a)
                                       : auctionLess(gh, key.column, a, b);
                });
            }
            return 0;
        }},
                // The browse tab's filter column, which was empty: it is built
                // from whatever GetAuctionItemClasses returns -
                // AuctionFrameBrowse_InitClasses copies the varargs straight
                // into CLASS_FILTERS - so answering nothing left the whole left
                // pane blank and the auction house searchable by name only.
                //
                // The lists are the ones this client's own auction window has
                // always shown, now shared, because the *position* in them is
                // the protocol: FrameXML hands the index back to
                // QueryAuctionItems and it has to mean the same thing on the
                // way in. Row zero is "All" and is not returned here - which
                // makes FrameXML's 1-based index land on the right row of the
                // shared table, and an unselected filter arrive as zero and
                // read as "any" without a special case.
                {"GetAuctionItemClasses", [](lua_State* L) -> int {
            for (int i = 1; i < game::kNumAuctionClasses; ++i)
                lua_pushstring(L, game::kAuctionClasses[i].label);
            return game::kNumAuctionClasses - 1;
        }},
                {"GetAuctionItemSubClasses", [](lua_State* L) -> int {
            const int ci = static_cast<int>(luaL_optnumber(L, 1, 0));
            if (ci < 1 || ci >= game::kNumAuctionClasses) return 0;
            int count = 0;
            const auto* subs = game::auctionSubsFor(game::kAuctionClasses[ci].classId, count);
            if (!subs) return 0;
            for (int i = 1; i < count; ++i) lua_pushstring(L, subs[i].label);
            return count - 1;
        }},
                // The third tier of the filter tree: where an item is worn.
                //
                // This answered nothing, on the grounds that the real client
                // has no such tier. It does - Armor, Cloth, Head - and with
                // nothing here the auction house could not be searched by slot
                // at all.
                //
                // Pairs, which is the shape AuctionFrameFilters_UpdateInvTypes
                // reads: the name of a global string, then a flag saying the
                // row is offered. The interface resolves the name itself, so
                // the wording is not written out a second time here.
                //
                // The number handed back is a position in kAuctionSlots, and
                // QueryAuctionItems reads that same table at that position -
                // so a renumbered subset would search a slot other than the
                // one clicked, silently. auctionSlotsFor answers positions for
                // exactly that reason.
                {"GetAuctionInvTypes", [](lua_State* L) -> int {
            const int ci = static_cast<int>(luaL_optnumber(L, 1, 0));
            const int si = static_cast<int>(luaL_optnumber(L, 2, 0));
            if (ci < 1 || ci >= game::kNumAuctionClasses) return 0;
            int count = 0;
            const uint8_t* slots =
                game::auctionSlotsFor(game::kAuctionClasses[ci].classId, si, count);
            if (!slots || count <= 0) return 0;
            // Two per row, and Lua guarantees room for twenty results without
            // being asked. Asking is what makes a longer list safe.
            if (!lua_checkstack(L, count * 2)) return 0;
            int pushed = 0;
            for (int i = 0; i < count; ++i) {
                const uint8_t at = slots[i];
                if (at >= game::kNumAuctionSlots) continue;
                lua_pushstring(L, game::kAuctionSlots[at].token);
                // The interface reads this only as "this row is offered"; the
                // row's own number is its position in what is returned here.
                lua_pushnumber(L, 1);
                pushed += 2;
            }
            return pushed;
        }},
                {"GetNumAuctionItems", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* listType = luaL_optstring(L, 1, "list");
            if (!gh) { lua_pushnumber(L, 0); lua_pushnumber(L, 0); return 2; }
            std::string t(listType);
            const game::AuctionListResult* r = nullptr;
            if (t == "list" || t == "browse") r = &gh->getAuctionBrowseResults();
            else if (t == "owner") r = &gh->getAuctionOwnerResults();
            else if (t == "bidder") r = &gh->getAuctionBidderResults();
            lua_pushnumber(L, r ? r->auctions.size() : 0);
            lua_pushnumber(L, r ? r->totalCount : 0);
            return 2;
        }},
                {"GetAuctionItemInfo", [](lua_State* L) -> int {
            // GetAuctionItemInfo(type, index) → name, texture, count, quality,
            // canUse, level, minBid, minIncrement, buyoutPrice, bidAmount,
            // highBidder, owner, saleStatus
            //
            // 3.3.5's thirteen, in 3.3.5's order. This answered Cataclysm's
            // seventeen - levelColHeader after level, and bidderFullName and
            // ownerFullName around owner - and the interface reading it is
            // 3.3.5's:
            //
            //   name, texture, count, quality, canUse, level, minBid,
            //   minIncrement, buyoutPrice, bidAmount, highBidder, owner
            //
            // so every value from the seventh on landed one place early. minBid
            // received the empty levelColHeader, minIncrement received minBid,
            // buyoutPrice received minIncrement - which is why no row showed a
            // buyout - bidAmount received the buyout, and owner received a
            // boolean. Bidding and buying read the prices they were given, so
            // both were computed from the wrong number and refused.
            auto* gh = getGameHandler(L);
            const char* listType = luaL_checkstring(L, 1);
            int index = static_cast<int>(luaL_checknumber(L, 2));
            if (!gh || index < 1) { return luaReturnNil(L); }
            std::string t(listType);
            const game::AuctionListResult* r = nullptr;
            if (t == "list") r = &gh->getAuctionBrowseResults();
            else if (t == "owner") r = &gh->getAuctionOwnerResults();
            else if (t == "bidder") r = &gh->getAuctionBidderResults();
            if (!r || index > static_cast<int>(r->auctions.size())) { return luaReturnNil(L); }
            const auto& a = r->auctions[index - 1];
            const auto* info = gh->getItemInfo(a.itemEntry);
            std::string name = info ? info->name : "Item #" + std::to_string(a.itemEntry);
            std::string icon = (info && info->displayInfoId != 0) ? gh->getItemIconPath(info->displayInfoId) : "";
            uint32_t quality = info ? info->quality : 1;
            lua_pushstring(L, name.c_str());        // name
            lua_pushstring(L, icon.empty() ? "Interface\\Icons\\INV_Misc_QuestionMark" : icon.c_str()); // texture
            lua_pushnumber(L, a.stackCount);        // count
            lua_pushnumber(L, quality);             // quality
            lua_pushboolean(L, 1);                  // canUse
            lua_pushnumber(L, info ? info->requiredLevel : 0); // level
            lua_pushnumber(L, a.startBid);          // minBid
            lua_pushnumber(L, a.minBidIncrement);   // minIncrement
            lua_pushnumber(L, a.buyoutPrice);       // buyoutPrice
            lua_pushnumber(L, a.currentBid);        // bidAmount
            // Whether *this player* holds the high bid, which is not the same
            // question as whether anybody does. It answered the second, so
            // every row with a bid on it - most of them - was labelled "Your
            // Bid", printed over the price it sits in front of.
            const bool isHighBidder =
                a.bidderGuid != 0 && a.bidderGuid == gh->getPlayerGuid();
            lua_pushboolean(L, isHighBidder ? 1 : 0); // highBidder
            // The seller column, which was two empty strings. The names are
            // asked for when the list arrives (handleAuctionListResult queries
            // every owner guid it does not know), so by the time a row is
            // drawn twice this has an answer; before that it is empty, which
            // is what an unknown seller looks like anyway.
            const std::string ownerName =
                a.ownerGuid ? gh->lookupName(a.ownerGuid) : std::string();
            lua_pushstring(L, ownerName.c_str());   // owner
            lua_pushnumber(L, 0);                   // saleStatus
            return 13;
        }},
                {"GetAuctionItemTimeLeft", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* listType = luaL_checkstring(L, 1);
            int index = static_cast<int>(luaL_checknumber(L, 2));
            if (!gh || index < 1) { lua_pushnumber(L, 4); return 1; }
            std::string t(listType);
            const game::AuctionListResult* r = nullptr;
            if (t == "list") r = &gh->getAuctionBrowseResults();
            else if (t == "owner") r = &gh->getAuctionOwnerResults();
            else if (t == "bidder") r = &gh->getAuctionBidderResults();
            if (!r || index > static_cast<int>(r->auctions.size())) { lua_pushnumber(L, 4); return 1; }
            // Return 1=short(<30m), 2=medium(<2h), 3=long(<12h), 4=very long(>12h)
            uint32_t ms = r->auctions[index - 1].timeLeftMs;
            int cat = (ms < 1800000) ? 1 : (ms < 7200000) ? 2 : (ms < 43200000) ? 3 : 4;
            lua_pushnumber(L, cat);
            return 1;
        }},
                {"GetAuctionItemLink", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const char* listType = luaL_checkstring(L, 1);
            int index = static_cast<int>(luaL_checknumber(L, 2));
            if (!gh || index < 1) { return luaReturnNil(L); }
            std::string t(listType);
            const game::AuctionListResult* r = nullptr;
            if (t == "list") r = &gh->getAuctionBrowseResults();
            else if (t == "owner") r = &gh->getAuctionOwnerResults();
            else if (t == "bidder") r = &gh->getAuctionBidderResults();
            if (!r || index > static_cast<int>(r->auctions.size())) { return luaReturnNil(L); }
            uint32_t itemId = r->auctions[index - 1].itemEntry;
            const auto* info = gh->getItemInfo(itemId);
            if (!info) { return luaReturnNil(L); }
        
    const std::string link = game::itemChatLink(itemId, static_cast<uint32_t>(info->quality), info->name);
            lua_pushstring(L, link.c_str());
            return 1;
        }},
                // Two values: how many are in the inbox, and how many the
                // server has in total. InboxFrame_Update compares them bare -
                // `if ( totalItems > numItems )`, to say more mail is waiting
                // than fits - so one value was an error every time mail
                // arrived. They are equal here: this client holds every mail it
                // has been sent, so none is waiting out of view.
                {"GetInboxNumItems", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const double n = gh ? static_cast<double>(gh->getMailInbox().size()) : 0.0;
            lua_pushnumber(L, n);
            lua_pushnumber(L, n);
            return 2;
        }},
                {"GetInboxHeaderInfo", [](lua_State* L) -> int {
            // GetInboxHeaderInfo(index) → packageIcon, stationeryIcon, sender, subject, money, COD, daysLeft, hasItem, wasRead, wasReturned, textCreated, canReply, isGM
            auto* gh = getGameHandler(L);
            int index = static_cast<int>(luaL_checknumber(L, 1));
            if (!gh || index < 1) { return luaReturnNil(L); }
            const auto& inbox = gh->getMailInbox();
            if (index > static_cast<int>(inbox.size())) { return luaReturnNil(L); }
            const auto& mail = inbox[index - 1];
            lua_pushstring(L, "Interface\\Icons\\INV_Letter_15"); // packageIcon
            lua_pushstring(L, "Interface\\Icons\\INV_Letter_15"); // stationeryIcon
            // Through getMailSenderName, which is what turns a player's guid
            // into a name once the query comes back, and names the auction
            // house, a creature or an object for the other message types. The
            // raw field is empty for player mail until then.
            lua_pushstring(L, gh->getMailSenderName(mail).c_str());  // sender
            const std::string subject = gh->getMailDisplaySubject(mail);
            lua_pushstring(L, subject.c_str());                   // subject
            lua_pushnumber(L, mail.money);                        // money (copper)
            lua_pushnumber(L, mail.cod);                          // COD
            lua_pushnumber(L, mail.expirationTime);              // daysLeft (server sends days)
            // A *count*, not a flag. InboxFrame_Update assigns it to both
            // button.hasItem and button.itemCount, and the tooltip then does
            //     MAIL_MULTIPLE_ITEMS.." ("..self.itemCount..")"
            // - concatenating a boolean raises, and `itemCount == 1` is never
            // true for one, so a single attachment took the multiple branch
            // and hovering any mail with something in it took the tooltip
            // down. Nil when empty, because zero is true in Lua and every
            // caller here tests it for truth.
            if (mail.attachments.empty()) lua_pushnil(L);
            else lua_pushnumber(L, static_cast<lua_Number>(mail.attachments.size()));
            lua_pushboolean(L, mail.read ? 1 : 0);               // wasRead
            lua_pushboolean(L, 0);                                // wasReturned
            lua_pushboolean(L, !mail.body.empty() ? 1 : 0);      // textCreated
            lua_pushboolean(L, mail.messageType == 0 ? 1 : 0);   // canReply (player mail only)
            lua_pushboolean(L, 0);                                // isGM
            // How many of the first attachment there are, which is what the
            // inbox button prints in its corner.
            lua_pushnumber(L, mail.attachments.empty()
                                  ? 0
                                  : static_cast<lua_Number>(mail.attachments[0].stackCount));
            return 14;
        }},
                // body, stationery texture, isTakeable, isInvoice
                //
                // The third decides whether the frame offers to take anything,
                // so a letter with coin or attachments in it needs to say so.
                // The stationery is answered with nothing: the id is known but
                // what art belongs to it is not, and a nil texture is an empty
                // background rather than a wrong one.
                // Reading a letter is what marks it read, and nothing did it.
                //
                // mailMarkAsRead had one caller - this client's own mail
                // window - and that window is not drawn once mail is handed
                // over. So a letter opened through the interface was never
                // marked, the envelope stayed bold, and HasNewMail answered
                // true for the rest of the session however much was read.
                //
                // Here rather than on the click, because this is the call that
                // means "the body is being shown": OpenMail_Update asks for it
                // as the letter opens, and the real client marks read at the
                // same point.
                {"GetInboxText", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            const auto* mail = mailAt(gh, static_cast<int>(luaL_optnumber(L, 1, 0)));
            if (!mail) { return luaReturnNil(L); }
            if (!mail->read && mail->messageId != 0) gh->mailMarkAsRead(mail->messageId);
            lua_pushstring(L, mail->body.c_str());
            lua_pushnil(L);
            lua_pushboolean(L, (mail->money > 0 || !mail->attachments.empty()) ? 1 : 0);
            // Whether this is an auction house invoice, which was answered no
            // for every letter. OpenMail_Update asks here first and only calls
            // GetInboxInvoiceInfo `if ( isInvoice )`, so saying no kept the
            // whole invoice panel shut: a sale arrived as a letter with the
            // raw colon-separated body in it and no breakdown at all.
            //
            // The body decides, not the sender: an auction mail's body parses
            // as an invoice and nothing else does.
            game::AuctionMailInvoice invoice;
            const bool isInvoice = mail->messageType == 2 &&
                                   game::parseAuctionMailBody(mail->body, invoice);
            lua_pushboolean(L, isInvoice ? 1 : 0);
            return 4;
        }},
                // Whether there is unread mail waiting, which is what the
                // envelope on the minimap reports.
                //
                // The server's flag first, because the inbox is only ever
                // filled while standing at a mailbox: away from one this list
                // is empty, and scanning it alone answered no to the only
                // question the envelope exists to ask. SMSG_RECEIVED_MAIL and
                // the next-mail-time poll are how a player standing in a field
                // finds out at all.
                //
                // The list still gets a look, for the other direction: at the
                // mailbox the flag is cleared on arrival, and what is unread
                // there is what the letters themselves say.
                {"HasNewMail", [](lua_State* L) -> int {
            auto* gh = getGameHandler(L);
            if (!gh) { return luaReturnFalse(L); }
            bool hasNew = gh->hasNewMail();
            if (!hasNew) {
                for (const auto& m : gh->getMailInbox()) {
                    if (!m.read) { hasNew = true; break; }
                }
            }
            lua_pushboolean(L, hasNew ? 1 : 0);
            return 1;
        }},
    };
    for (const auto& [name, func] : api) {
        lua_pushcfunction(L, func);
        lua_setglobal(L, name);
    }
}

} // namespace wowee::addons
