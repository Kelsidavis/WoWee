#include "game/inventory_handler.hpp"
#include "game/game_handler.hpp"
#include "game/game_utils.hpp"
#include "game/entity.hpp"
#include "game/packet_parsers.hpp"
#include "rendering/renderer.hpp"
#include "audio/audio_coordinator.hpp"
#include "audio/ui_sound_manager.hpp"
#include "audio/player_voice_manager.hpp"
#include "core/application.hpp"
#include "core/logger.hpp"
#include "network/world_socket.hpp"
#include "network/packet.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/dbc_loader.hpp"
#include "pipeline/dbc_layout.hpp"
#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstdlib>
#include <sstream>

namespace wowee {
namespace game {

std::string formatCopperAmount(uint32_t amount);

InventoryHandler::InventoryHandler(GameHandler& owner)
    : owner_(owner) {}

namespace {
constexpr uint32_t kItemClassConsumable = 0;
constexpr uint32_t kConsumableSubclassBandage = 7;
constexpr uint32_t kConsumableSubclassItemEnhancement = 6;
// SpellCastTargetFlags bit set by Spell.dbc for spells cast onto another item.
constexpr uint32_t kSpellTargetFlagItem = 0x10;
constexpr uint32_t kBuybackWireSlotStart = 74;
constexpr uint32_t kBuybackWireSlotCount = 12;
constexpr uint32_t kBuybackWireSlotEnd =
    kBuybackWireSlotStart + kBuybackWireSlotCount;

bool isBandageItem(const ItemQueryResponseData* info) {
    return info && info->valid &&
           info->itemClass == kItemClassConsumable &&
           info->subClass == kConsumableSubclassBandage;
}

void synchronizeStationaryBandageCast(GameHandler& owner) {
    // Bandages are cast through CMSG_USE_ITEM and therefore bypass
    // SpellHandler::castSpell(), which normally sends a stop before a timed cast.
    // Explicitly synchronize the stationary state so the server cannot retain a
    // stale start-forward/strafe/turn state after the character has visually stopped.
    auto& movement = owner.movementInfoRef();
    const uint32_t horizontalMask =
        static_cast<uint32_t>(MovementFlags::FORWARD) |
        static_cast<uint32_t>(MovementFlags::BACKWARD) |
        static_cast<uint32_t>(MovementFlags::STRAFE_LEFT) |
        static_cast<uint32_t>(MovementFlags::STRAFE_RIGHT);
    const uint32_t turnMask =
        static_cast<uint32_t>(MovementFlags::TURN_LEFT) |
        static_cast<uint32_t>(MovementFlags::TURN_RIGHT);
    movement.flags &= ~(horizontalMask | turnMask);

    owner.sendMovement(Opcode::MSG_MOVE_STOP);
    owner.sendMovement(Opcode::MSG_MOVE_STOP_STRAFE);
    owner.sendMovement(Opcode::MSG_MOVE_STOP_TURN);
    owner.sendMovement(Opcode::MSG_MOVE_HEARTBEAT);
}

bool usesVisibleItemDisplayIds() {
    return false;
}

constexpr int kTbcVisibleItemStride = 16;

int tbcVisibleItemBaseFallback() {
    const uint16_t invSlotHead = fieldIndex(UF::PLAYER_FIELD_INV_SLOT_HEAD);
    constexpr int kVisibleSlots = 19;
    const int visibleBytes = kVisibleSlots * kTbcVisibleItemStride;
    if (invSlotHead != 0xFFFF && invSlotHead >= visibleBytes) {
        return static_cast<int>(invSlotHead) - visibleBytes;
    }
    return 346;
}

bool visibleEquipmentFieldDiagEnabled() {
    static const bool enabled = [] {
        const char* raw = std::getenv("WOWEE_VISIBLE_EQUIP_FIELD_DIAG");
        if (!raw || raw[0] == '\0') return false;
        std::string value(raw);
        std::transform(value.begin(), value.end(), value.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        return value != "0" && value != "false" && value != "off";
    }();
    return enabled;
}

std::array<uint8_t, 19> inferredVisibleInventoryTypes() {
    return {
        InvType::HEAD,
        InvType::NECK,
        InvType::SHOULDERS,
        InvType::SHIRT,
        InvType::CHEST,
        InvType::WAIST,
        InvType::LEGS,
        InvType::FEET,
        InvType::WRISTS,
        InvType::HANDS,
        InvType::FINGER,
        InvType::FINGER,
        InvType::TRINKET,
        InvType::TRINKET,
        InvType::BACK,
        InvType::MAIN_HAND,
        InvType::SHIELD,
        InvType::RANGED_BOW,
        InvType::TABARD,
    };
}

uint64_t targetGuidForUseItem(GameHandler& owner, const ItemQueryResponseData* info) {
    if (!info || !info->valid || info->itemClass != kItemClassConsumable) return 0;
    if (isBandageItem(info)) {
        return owner.getPlayerGuid();
    }
    if (info->subClass == kConsumableSubclassItemEnhancement) return 0;
    return owner.getPlayerGuid();
}
} // namespace

// ============================================================
// Opcode Registration
// ============================================================

void InventoryHandler::registerOpcodes(DispatchTable& table) {
    // ---- Item query response ----
    table[Opcode::SMSG_ITEM_QUERY_SINGLE_RESPONSE] = [this](network::Packet& packet) { handleItemQueryResponse(packet); };
    table[Opcode::SMSG_ITEM_QUERY_MULTIPLE_RESPONSE] = [this](network::Packet& packet) { handleItemQueryResponse(packet); };

    // ---- Loot ----
    table[Opcode::SMSG_LOOT_RESPONSE] = [this](network::Packet& packet) { handleLootResponse(packet); };
    table[Opcode::SMSG_LOOT_RELEASE_RESPONSE] = [this](network::Packet& packet) { handleLootReleaseResponse(packet); };
    table[Opcode::SMSG_LOOT_REMOVED] = [this](network::Packet& packet) { handleLootRemoved(packet); };
    table[Opcode::SMSG_LOOT_ROLL] = [this](network::Packet& packet) { handleLootRoll(packet); };
    table[Opcode::SMSG_LOOT_ROLL_WON] = [this](network::Packet& packet) { handleLootRollWon(packet); };
    table[Opcode::SMSG_LOOT_MASTER_LIST] = [this](network::Packet& packet) {
        masterLootCandidates_.clear();
        if (!packet.hasRemaining(1)) return;
        uint8_t mlCount = packet.readUInt8();
        masterLootCandidates_.reserve(mlCount);
        for (uint8_t i = 0; i < mlCount; ++i) {
            if (!packet.hasRemaining(8)) break;
            masterLootCandidates_.push_back(packet.readUInt64());
        }
    };

    // ---- Loot money / misc consume ----
    table[Opcode::SMSG_LOOT_MONEY_NOTIFY] = [this](network::Packet& packet) {
        if (!packet.hasRemaining(4)) return;
        uint32_t amount = packet.readUInt32();
        if (packet.hasRemaining(1))
            /*uint8_t soleLooter =*/ packet.readUInt8();
        owner_.playerMoneyCopperRef() += amount;
        owner_.pendingMoneyDeltaRef() = amount;
        owner_.pendingMoneyDeltaTimerRef() = 2.0f;
        uint64_t notifyGuid = pendingLootMoneyGuid_ != 0 ? pendingLootMoneyGuid_ : currentLoot_.lootGuid;
        pendingLootMoneyGuid_ = 0;
        pendingLootMoneyAmount_ = 0;
        pendingLootMoneyNotifyTimer_ = 0.0f;
        bool alreadyAnnounced = false;
        auto it = localLootState_.find(notifyGuid);
        if (it != localLootState_.end()) {
            alreadyAnnounced = it->second.moneyTaken;
            it->second.moneyTaken = true;
        }
        if (!alreadyAnnounced) {
            owner_.addSystemChatMessage("Looted: " + formatCopperAmount(amount));
            auto* ac = owner_.services().audioCoordinator;
            if (ac) {
                if (auto* sfx = ac->getUiSoundManager()) {
                    if (amount >= 10000) sfx->playLootCoinLarge();
                    else sfx->playLootCoinSmall();
                }
            }
            if (notifyGuid != 0)
                recentLootMoneyAnnounceCooldowns_[notifyGuid] = 1.5f;
        }
        if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("PLAYER_MONEY", {});
    };
    table[Opcode::SMSG_LOOT_CLEAR_MONEY] = [](network::Packet& /*packet*/) {};

    // ---- Read item (books) (moved from GameHandler) ----
    table[Opcode::SMSG_READ_ITEM_OK] = [](network::Packet& packet) {
        packet.skipAll();
    };
    table[Opcode::SMSG_READ_ITEM_FAILED] = [this](network::Packet& packet) {
        owner_.addUIError("You cannot read this item.");
        owner_.addSystemChatMessage("You cannot read this item.");
        packet.skipAll();
    };

    // ---- Loot roll start / notifications ----
    table[Opcode::SMSG_LOOT_START_ROLL] = [this](network::Packet& packet) {
        // objectGuid(8) + mapId(4) (WotLK) + lootSlot(4) + itemId(4) + randSuffix(4) +
        // randProp(4) + countdown(4) + voteMask(1)
        const bool hasMapId = isActiveExpansion("wotlk");
        const size_t minSz = hasMapId ? 33 : 29;
        if (packet.getRemainingSize() < minSz) return;
        uint64_t objectGuid = packet.readUInt64();
        if (hasMapId) packet.readUInt32(); // mapId
        uint32_t lootSlot = packet.readUInt32();
        uint32_t itemId   = packet.readUInt32();
        /*uint32_t randSuffix =*/ packet.readUInt32();
        (void)packet.readUInt32(); // random property
        uint32_t countdown = packet.readUInt32();
        uint8_t  voteMask  = packet.readUInt8();

        // Resolve item name from cache
        owner_.ensureItemInfo(itemId);
        auto* info = owner_.getItemInfo(itemId);
        std::string itemName = (info && !info->name.empty()) ? info->name : ("Item #" + std::to_string(itemId));
        uint8_t quality = info ? static_cast<uint8_t>(info->quality) : 1;

        pendingLootRollActive_ = true;
        pendingLootRoll_ = {};
        pendingLootRoll_.objectGuid = objectGuid;
        pendingLootRoll_.slot = lootSlot;
        pendingLootRoll_.itemId = itemId;
        pendingLootRoll_.itemName = itemName;
        pendingLootRoll_.itemQuality = quality;
        pendingLootRoll_.rollCountdownMs = countdown;
        pendingLootRoll_.voteMask = voteMask;
        pendingLootRoll_.rollStartedAt = std::chrono::steady_clock::now();
        pendingLootRoll_.playerRolls.clear();
        std::string link = buildItemLink(itemId, quality, itemName);
        owner_.addSystemChatMessage("Loot roll started for " + link + ".");
        if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("START_LOOT_ROLL", {});
    };

    table[Opcode::SMSG_LOOT_ALL_PASSED] = [this](network::Packet& packet) {
        // objectGuid(8) + lootSlot(4) + itemId(4) + randSuffix(4) + randProp(4)
        if (!packet.hasRemaining(24)) return;
        /*uint64_t objectGuid =*/ packet.readUInt64();
        /*uint32_t lootSlot   =*/ packet.readUInt32();
        uint32_t itemId     = packet.readUInt32();
        /*uint32_t randSuffix =*/ packet.readUInt32();
        (void)packet.readUInt32(); // random property
        owner_.ensureItemInfo(itemId);
        auto* allPassInfo = owner_.getItemInfo(itemId);
        std::string allPassName = (allPassInfo && !allPassInfo->name.empty())
            ? allPassInfo->name : ("Item #" + std::to_string(itemId));
        uint32_t allPassQuality = allPassInfo ? allPassInfo->quality : 1u;
        owner_.addSystemChatMessage("Everyone passed on " + buildItemLink(itemId, allPassQuality, allPassName) + ".");
        pendingLootRollActive_ = false;
    };

    table[Opcode::SMSG_LOOT_ITEM_NOTIFY] = [this](network::Packet& packet) {
        // objectGuid(8) + lootSlot(1) [Classic: uint32; WotLK: uint8]
        if (!packet.hasRemaining(9)) return;
        /*uint64_t objectGuid =*/ packet.readUInt64();
        uint32_t lootSlot;
        if (isActiveExpansion("wotlk")) {
            lootSlot = packet.readUInt8();
        } else {
            if (!packet.hasRemaining(4)) return;
            lootSlot = packet.readUInt32();
        }
        // Try to resolve item name
        uint32_t itemId = 0;
        for (const auto& lootItem : currentLoot_.items) {
            if (lootItem.slotIndex == static_cast<uint8_t>(lootSlot)) {
                itemId = lootItem.itemId;
                break;
            }
        }
        if (itemId != 0) {
            auto* notifInfo = owner_.getItemInfo(itemId);
            std::string itemName = (notifInfo && !notifInfo->name.empty())
                ? notifInfo->name : ("Item #" + std::to_string(itemId));
            uint32_t notifyQuality = notifInfo ? notifInfo->quality : 1u;
            std::string itemLink2 = buildItemLink(itemId, notifyQuality, itemName);
            owner_.addSystemChatMessage("You receive loot: " + itemLink2 + ".");
        }
    };

    table[Opcode::SMSG_LOOT_SLOT_CHANGED] = [](network::Packet& packet) {
        if (packet.hasRemaining(1)) {
            uint8_t slotIdx = packet.readUInt8();
            LOG_DEBUG("SMSG_LOOT_SLOT_CHANGED: slot=", (int)slotIdx);
            // The server re-sends loot info for this slot; we can refresh from
            // the next SMSG_LOOT_RESPONSE or SMSG_LOOT_ITEM_NOTIFY.
        }
    };

    // ---- Item push result ----
    table[Opcode::SMSG_ITEM_PUSH_RESULT] = [this](network::Packet& packet) {
        // WotLK 3.3.5a: guid(8)+received(4)+created(4)+displayInChat(4)+bagSlot(1)
        //   +slot(4)+itemId(4)+suffixFactor(4)+randomPropertyId(4)+count(4)+countInInventory(4)
        if (!packet.hasRemaining(45)) return;
        uint64_t guid = packet.readUInt64();
        if (guid != owner_.getPlayerGuid()) { packet.skipAll(); return; }
        /*uint32_t received      =*/ packet.readUInt32();
        /*uint32_t created       =*/ packet.readUInt32();
        /*uint32_t displayInChat =*/ packet.readUInt32();
        /*uint8_t  bagSlot       =*/ packet.readUInt8();
        /*uint32_t slot          =*/ packet.readUInt32();
        uint32_t itemId = packet.readUInt32();
        /*uint32_t suffixFactor  =*/ packet.readUInt32();
        int32_t randomProp = static_cast<int32_t>(packet.readUInt32());
        uint32_t count = packet.readUInt32();

        auto* info = owner_.getItemInfo(itemId);
        if (!info || info->name.empty()) {
            // Item info not yet cached — defer notification
            owner_.pendingItemPushNotifsRef().push_back({itemId, count});
            owner_.ensureItemInfo(itemId);
            return;
        }
        std::string itemName = info->name;
        if (randomProp != 0) {
            std::string suffix = owner_.getRandomPropertyName(randomProp);
            if (!suffix.empty()) itemName += " " + suffix;
        }
        uint32_t quality = info->quality;
        std::string link = buildItemLink(itemId, quality, itemName);
        std::string msg = "Received item: " + link;
        if (count > 1) msg += " x" + std::to_string(count);
        owner_.addSystemChatMessage(msg);
        if (auto* ac = owner_.services().audioCoordinator) {
            if (auto* sfx = ac->getUiSoundManager())
                sfx->playLootItem();
        }
        if (owner_.addonEventCallbackRef()) {
            owner_.addonEventCallbackRef()("BAG_UPDATE", {});
            owner_.addonEventCallbackRef()("ITEM_PUSH", {std::to_string(itemId), std::to_string(count)});
        }
        if (owner_.itemLootCallbackRef())
            owner_.itemLootCallbackRef()(itemId, count, quality, itemName);
    };

    // ---- Open container ----
    table[Opcode::SMSG_OPEN_CONTAINER] = [](network::Packet& packet) {
        if (packet.hasRemaining(8)) {
            uint64_t containerGuid = packet.readUInt64();
            LOG_DEBUG("SMSG_OPEN_CONTAINER: guid=0x", std::hex, containerGuid, std::dec);
        }
    };

    // ---- Sell / Buy / Inventory ----
    table[Opcode::SMSG_SELL_ITEM] = [this](network::Packet& packet) {
        if ((packet.getRemainingSize()) >= 17) {
            uint64_t vendorGuid = packet.readUInt64();
            uint64_t itemGuid = packet.readUInt64();
            uint8_t result = packet.readUInt8();
            LOG_INFO("SMSG_SELL_ITEM: vendorGuid=0x", std::hex, vendorGuid,
                     " itemGuid=0x", itemGuid, std::dec,
                     " result=", static_cast<int>(result));
            if (result == 0) {
                auto pending = pendingSellToBuyback_.find(itemGuid);
                if (pending != pendingSellToBuyback_.end()) {
                    pendingSellToBuyback_.erase(pending);
                    reconcileBuybackSlots();
                } else {
                    LOG_WARNING("Successful sale had no pending buyback entry: itemGuid=0x",
                                std::hex, itemGuid, std::dec);
                }
                if (auto* ac = owner_.services().audioCoordinator) {
                    if (auto* sfx = ac->getUiSoundManager())
                        sfx->playDropOnGround();
                }
                if (owner_.addonEventCallbackRef()) {
                    owner_.addonEventCallbackRef()("BAG_UPDATE", {});
                    owner_.addonEventCallbackRef()("PLAYER_MONEY", {});
                }
            } else {
                auto it = pendingSellToBuyback_.find(itemGuid);
                if (it != pendingSellToBuyback_.end()) {
                    pendingSellToBuyback_.erase(it);
                }
                const auto rejected = std::find_if(
                    buybackItems_.begin(), buybackItems_.end(),
                    [itemGuid](const BuybackItem& item) {
                        return item.itemGuid == itemGuid;
                    });
                if (rejected != buybackItems_.end()) buybackItems_.erase(rejected);
                static const char* sellErrors[] = {
                    "OK", "Can't find item", "Can't sell item",
                    "Can't find vendor", "You don't own that item",
                    "Unknown error", "Only empty bag"
                };
                const char* msg = (result < 7) ? sellErrors[result] : "Unknown sell error";
                owner_.addUIError(std::string("Sell failed: ") + msg);
                owner_.addSystemChatMessage(std::string("Sell failed: ") + msg);
                if (auto* ac = owner_.services().audioCoordinator) {
                    if (auto* sfx = ac->getUiSoundManager())
                        sfx->playError();
                }
                LOG_WARNING("SMSG_SELL_ITEM error: ", (int)result, " (", msg,
                            ") itemGuid=0x", std::hex, itemGuid,
                            " vendorGuid=0x", vendorGuid, std::dec);
            }
        }
    };

    table[Opcode::SMSG_INVENTORY_CHANGE_FAILURE] = [this](network::Packet& packet) {
        if (packet.getRemainingSize() < 1) return;
        uint8_t error = packet.readUInt8();
        if (error == 0) return;

        LOG_WARNING("SMSG_INVENTORY_CHANGE_FAILURE: error=", (int)error);
        uint32_t requiredLevel = 0;
        if (packet.hasRemaining(17)) {
            packet.readUInt64();
            packet.readUInt64();
            packet.readUInt8();
            if (error == 1 && packet.hasRemaining(4))
                requiredLevel = packet.readUInt32();
        }

        // Level requirement has its own formatting
        if (error == 1) {
            char levelBuf[64];
            if (requiredLevel > 0) {
                std::snprintf(levelBuf, sizeof(levelBuf),
                              "You must reach level %u to use that item.", requiredLevel);
            } else {
                std::snprintf(levelBuf, sizeof(levelBuf),
                              "You must reach a higher level to use that item.");
            }
            owner_.addUIError(levelBuf);
            owner_.addSystemChatMessage(levelBuf);
            owner_.playErrorSpeech(audio::PlayerErrorSpeech::CANT_EQUIP_LEVEL);
            return;
        }

        const char* errMsg = nullptr;
        switch (error) {
                    case 3:  errMsg = "That item doesn't go in that slot."; break;
                    case 4:  errMsg = "That bag is full."; break;
                    case 5:  errMsg = "Can't put bags in bags."; break;
                    case 6:  errMsg = "Can't trade equipped bags."; break;
                    case 7:  errMsg = "That slot only holds ammo."; break;
                    case 8:  errMsg = "You can't use that item."; break;
                    case 9:  errMsg = "No equipment slot available."; break;
                    case 10: errMsg = "You can never use that item."; break;
                    case 11: errMsg = "You can never use that item."; break;
                    case 12: errMsg = "No equipment slot available."; break;
                    case 13: errMsg = "Can't equip with a two-handed weapon."; break;
                    case 14: errMsg = "Can't dual-wield."; break;
                    case 15: errMsg = "That item doesn't go in that bag."; break;
                    case 16: errMsg = "That item doesn't go in that bag."; break;
                    case 17: errMsg = "You can't carry any more of those."; break;
                    case 18: errMsg = "No equipment slot available."; break;
                    case 19: errMsg = "Can't stack those items."; break;
                    case 20: errMsg = "That item can't be equipped."; break;
                    case 21: errMsg = "Can't swap items."; break;
                    case 22: errMsg = "That slot is empty."; break;
                    case 23: errMsg = "Item not found."; break;
                    case 24: errMsg = "Can't drop soulbound items."; break;
                    case 25: errMsg = "Out of range."; break;
                    case 26: errMsg = "Need to split more than 1."; break;
                    case 27: errMsg = "Split failed."; break;
                    case 28: errMsg = "Not enough reagents."; break;
                    case 29: errMsg = "Not enough money."; break;
                    case 30: errMsg = "Not a bag."; break;
                    case 31: errMsg = "Can't destroy non-empty bag."; break;
                    case 32: errMsg = "You don't own that item."; break;
                    case 33: errMsg = "You can only have one quiver."; break;
                    case 34: errMsg = "No free bank slots."; break;
                    case 35: errMsg = "No bank here."; break;
                    case 36: errMsg = "Item is locked."; break;
                    case 37: errMsg = "You are stunned."; break;
                    case 38: errMsg = "You are dead."; break;
                    case 39: errMsg = "Can't do that right now."; break;
                    case 40: errMsg = "Internal bag error."; break;
                    case 49: errMsg = "Loot is gone."; break;
                    case 50: errMsg = "Inventory is full."; break;
                    case 51: errMsg = "Bank is full."; break;
                    case 52: errMsg = "That item is sold out."; break;
                    case 58: errMsg = "That object is busy."; break;
                    case 60: errMsg = "Can't do that in combat."; break;
                    case 61: errMsg = "Can't do that while disarmed."; break;
                    case 63: errMsg = "Requires a higher rank."; break;
                    case 64: errMsg = "Requires higher reputation."; break;
                    case 67: errMsg = "That item is unique-equipped."; break;
                    case 69: errMsg = "Not enough honor points."; break;
                    case 70: errMsg = "Not enough arena points."; break;
                    case 77: errMsg = "Too much gold."; break;
                    case 78: errMsg = "Can't do that during arena match."; break;
                    case 80: errMsg = "Requires a personal arena rating."; break;
                    case 87: errMsg = "Requires a higher level."; break;
                    case 88: errMsg = "Requires the right talent."; break;
                    default: break;
                }
        std::string msg = errMsg ? errMsg : "Inventory error (" + std::to_string(error) + ").";
        if (error == 50 && currentLoot_.lootGuid != 0) {
            msg = "Cannot loot item: Inventory is full.";
        }
        owner_.addUIError(msg);
        owner_.addSystemChatMessage(msg);
        if (auto* ac = owner_.services().audioCoordinator) {
            if (auto* sfx = ac->getUiSoundManager())
                sfx->playError();
        }

        // Character speech response for errors with a matching voice line
        using audio::PlayerErrorSpeech;
        switch (error) {
            case 4:  owner_.playErrorSpeech(PlayerErrorSpeech::BAG_FULL); break;
            case 7:  owner_.playErrorSpeech(PlayerErrorSpeech::AMMO_ONLY); break;
            case 8:  owner_.playErrorSpeech(PlayerErrorSpeech::CANT_USE_ITEM); break;
            case 10:
            case 11: owner_.playErrorSpeech(PlayerErrorSpeech::CANT_EQUIP_EVER); break;
            case 17: owner_.playErrorSpeech(PlayerErrorSpeech::ITEM_MAX_COUNT); break;
            case 20: owner_.playErrorSpeech(PlayerErrorSpeech::NOT_EQUIPPABLE); break;
            case 24: owner_.playErrorSpeech(PlayerErrorSpeech::CANT_DROP_SOULBOUND); break;
            case 25: owner_.playErrorSpeech(PlayerErrorSpeech::OUT_OF_RANGE); break;
            case 29: owner_.playErrorSpeech(PlayerErrorSpeech::NOT_ENOUGH_MONEY); break;
            case 30: owner_.playErrorSpeech(PlayerErrorSpeech::NOT_A_BAG); break;
            case 36: owner_.playErrorSpeech(PlayerErrorSpeech::ITEM_LOCKED); break;
            case 50:
            case 51: owner_.playErrorSpeech(PlayerErrorSpeech::INVENTORY_FULL); break;
            default: break;
        }
    };

    table[Opcode::SMSG_BUY_FAILED] = [this](network::Packet& packet) {
        if (packet.hasRemaining(13)) {
            uint64_t vendorGuid = packet.readUInt64();
            uint32_t itemIdOrSlot = packet.readUInt32();
            uint8_t errCode = packet.readUInt8();
            LOG_INFO("SMSG_BUY_FAILED: vendorGuid=0x", std::hex, vendorGuid, std::dec,
                     " item/slot=", itemIdOrSlot,
                     " err=", static_cast<int>(errCode),
                     " pendingBuybackSlot=", pendingBuybackSlot_,
                     " pendingBuybackWireSlot=", pendingBuybackWireSlot_,
                     " pendingBuyItemId=", pendingBuyItemId_,
                     " pendingBuyItemSlot=", pendingBuyItemSlot_);
            if (pendingBuybackSlot_ >= 0) {
                if (errCode == 0) {
                    // A missing slot is stale local state.  Never scan adjacent
                    // slots: another item may legitimately occupy them.
                    const auto stale = std::find_if(
                        buybackItems_.begin(), buybackItems_.end(),
                        [this](const BuybackItem& item) {
                            return item.wireSlot == pendingBuybackWireSlot_;
                        });
                    if (stale != buybackItems_.end()) buybackItems_.erase(stale);
                    pendingBuybackSlot_ = -1;
                    pendingBuybackWireSlot_ = 0;
                    if (currentVendorItems_.vendorGuid != 0 && owner_.getSocket() && owner_.getState() == WorldState::IN_WORLD) {
                        auto pkt = ListInventoryPacket::build(currentVendorItems_.vendorGuid);
                        owner_.getSocket()->send(pkt);
                    }
                    owner_.addUIError("That buyback item is no longer available.");
                    return;
                }
                pendingBuybackSlot_ = -1;
                pendingBuybackWireSlot_ = 0;
            }
            const char* msg = "Purchase failed.";
            switch (errCode) {
                case 0: msg = "Purchase failed: item not found."; break;
                case 2: msg = "You don't have enough money."; break;
                case 4: msg = "Seller is too far away."; break;
                case 5: msg = "That item is sold out."; break;
                case 6: msg = "You can't carry any more items."; break;
                default: break;
            }
            owner_.addUIError(msg);
            owner_.addSystemChatMessage(msg);
            if (auto* ac = owner_.services().audioCoordinator) {
                if (auto* sfx = ac->getUiSoundManager())
                    sfx->playError();
            }
            if (errCode == 2)
                owner_.playErrorSpeech(audio::PlayerErrorSpeech::NOT_ENOUGH_MONEY);
            else if (errCode == 6)
                owner_.playErrorSpeech(audio::PlayerErrorSpeech::INVENTORY_FULL);
        }
    };

    table[Opcode::SMSG_BUY_ITEM] = [this](network::Packet& packet) {
        if (packet.hasRemaining(20)) {
            /*uint64_t vendorGuid =*/ packet.readUInt64();
            /*uint32_t vendorSlot =*/ packet.readUInt32();
            (void)packet.readUInt32(); // new count
            uint32_t itemCount  = packet.readUInt32();
            // Successful buyback: remove the entry from the local buyback list.
            // Without this the pending slot lingered and a later unrelated
            // SMSG_BUY_FAILED could misread it as a buyback retry.
            if (pendingBuybackSlot_ >= 0) {
                const auto bought = std::find_if(
                    buybackItems_.begin(), buybackItems_.end(),
                    [this](const BuybackItem& item) {
                        return item.wireSlot == pendingBuybackWireSlot_;
                    });
                if (bought != buybackItems_.end()) {
                    const auto& entry = *bought;
                    std::string label = entry.item.name.empty()
                        ? "item #" + std::to_string(entry.item.itemId) : entry.item.name;
                    owner_.addSystemChatMessage("Bought back: " +
                        buildItemLink(entry.item.itemId,
                                      static_cast<uint32_t>(entry.item.quality), label));
                    buybackItems_.erase(bought);
                }
                pendingBuybackSlot_ = -1;
                pendingBuybackWireSlot_ = 0;
                if (auto* ac = owner_.services().audioCoordinator) {
                    if (auto* sfx = ac->getUiSoundManager())
                        sfx->playPickupBag();
                }
                if (owner_.addonEventCallbackRef()) {
                    owner_.addonEventCallbackRef()("MERCHANT_UPDATE", {});
                    owner_.addonEventCallbackRef()("BAG_UPDATE", {});
                }
                return;
            }
            if (pendingBuyItemId_ != 0) {
                std::string itemLabel;
                uint32_t buyQuality = 1;
                if (const ItemQueryResponseData* info = owner_.getItemInfo(pendingBuyItemId_)) {
                    if (!info->name.empty()) itemLabel = info->name;
                    buyQuality = info->quality;
                }
                if (itemLabel.empty()) itemLabel = "item #" + std::to_string(pendingBuyItemId_);
                std::string msg = "Purchased: " + buildItemLink(pendingBuyItemId_, buyQuality, itemLabel);
                if (itemCount > 1) msg += " x" + std::to_string(itemCount);
                owner_.addSystemChatMessage(msg);
                if (auto* ac = owner_.services().audioCoordinator) {
                    if (auto* sfx = ac->getUiSoundManager())
                        sfx->playPickupBag();
                }
            }
            pendingBuyItemId_   = 0;
            pendingBuyItemSlot_ = 0;
            if (owner_.addonEventCallbackRef()) {
                owner_.addonEventCallbackRef()("MERCHANT_UPDATE", {});
                owner_.addonEventCallbackRef()("BAG_UPDATE", {});
            }
        }
    };

    // ---- Vendor / Trainer ----
    table[Opcode::SMSG_LIST_INVENTORY] = [this](network::Packet& packet) { handleListInventory(packet); };
    table[Opcode::SMSG_TRAINER_LIST] = [this](network::Packet& packet) { handleTrainerList(packet); };

    // ---- Mail ----
    table[Opcode::SMSG_SHOW_MAILBOX] = [this](network::Packet& packet) { handleShowMailbox(packet); };
    table[Opcode::SMSG_MAIL_LIST_RESULT] = [this](network::Packet& packet) { handleMailListResult(packet); };
    table[Opcode::SMSG_SEND_MAIL_RESULT] = [this](network::Packet& packet) { handleSendMailResult(packet); };
    table[Opcode::SMSG_RECEIVED_MAIL] = [this](network::Packet& packet) { handleReceivedMail(packet); };
    table[Opcode::MSG_QUERY_NEXT_MAIL_TIME] = [this](network::Packet& packet) { handleQueryNextMailTime(packet); };

    // ---- Bank ----
    table[Opcode::SMSG_SHOW_BANK] = [this](network::Packet& packet) { handleShowBank(packet); };
    table[Opcode::SMSG_BUY_BANK_SLOT_RESULT] = [this](network::Packet& packet) { handleBuyBankSlotResult(packet); };

    // ---- Guild Bank ----
    table[Opcode::SMSG_GUILD_BANK_LIST] = [this](network::Packet& packet) { handleGuildBankList(packet); };

    // ---- Auction House ----
    table[Opcode::MSG_AUCTION_HELLO] = [this](network::Packet& packet) { handleAuctionHello(packet); };
    table[Opcode::SMSG_AUCTION_LIST_RESULT] = [this](network::Packet& packet) { handleAuctionListResult(packet); };
    table[Opcode::SMSG_AUCTION_OWNER_LIST_RESULT] = [this](network::Packet& packet) { handleAuctionOwnerListResult(packet); };
    table[Opcode::SMSG_AUCTION_BIDDER_LIST_RESULT] = [this](network::Packet& packet) { handleAuctionBidderListResult(packet); };
    table[Opcode::SMSG_AUCTION_COMMAND_RESULT] = [this](network::Packet& packet) { handleAuctionCommandResult(packet); };

    table[Opcode::SMSG_AUCTION_OWNER_NOTIFICATION] = [this](network::Packet& packet) {
        // SendAuctionOwnerNotification writes:
        //   auctionId(4) bid(4) unk(4) unkGuid(8) item_template(4) unk(4)
        //   unkTime(float 4) = 32 bytes.
        // Sent only when an owned auction SELLS — expiry and outbids arrive on
        // their own opcodes (SMSG_AUCTION_REMOVED_NOTIFICATION /
        // SMSG_AUCTION_BIDDER_NOTIFICATION).
        //
        // The item entry is at offset 20 and reading field 3 made every "sold"
        // line say "Item #0". That was fixed by counting six uint32s to reach
        // twenty, which lands in the right place and describes the packet
        // wrongly: the twelve bytes in the middle are one eight-byte guid and
        // one word, not three words. Written out properly here so the next
        // person to add a field does not count from a layout that is not the
        // one on the wire.
        if (packet.hasRemaining(24)) {
            /*uint32_t auctionId =*/ packet.readUInt32();
            /*uint32_t bid       =*/ packet.readUInt32();
            /*uint32_t unk       =*/ packet.readUInt32();
            /*uint64_t unkGuid   =*/ packet.readUInt64();
            uint32_t itemEntry = packet.readUInt32();
            owner_.ensureItemInfo(itemEntry);
            auto* info = owner_.getItemInfo(itemEntry);
            std::string rawName = info && !info->name.empty() ? info->name : ("Item #" + std::to_string(itemEntry));
            uint32_t aucQuality = info ? info->quality : 1u;
            std::string itemLink = buildItemLink(itemEntry, aucQuality, rawName);
            owner_.addSystemChatMessage("Your auction of " + itemLink + " has sold!");
        }
        packet.skipAll();
    };

    table[Opcode::SMSG_AUCTION_BIDDER_NOTIFICATION] = [this](network::Packet& packet) {
        // WotLK format: auctionHouseId(4) + auctionId(4) + bidderGuid(8) +
        // bidAmount(4) + outbidAmount(4) + itemEntry(4) + randomPropertyId(4) = 32 bytes.
        // Previously read auctionHouseId as auctionId and auctionId as itemEntry,
        // so outbid messages always referenced the wrong item.
        if (!packet.hasRemaining(32)) { packet.skipAll(); return; }
        /*uint32_t auctionHouseId =*/ packet.readUInt32();
        /*uint32_t auctionId      =*/ packet.readUInt32();
        /*uint64_t bidderGuid     =*/ packet.readUInt64();
        /*uint32_t bidAmount      =*/ packet.readUInt32();
        /*uint32_t outbidAmount   =*/ packet.readUInt32();
        uint32_t itemEntry = packet.readUInt32();
        int32_t bidRandProp = static_cast<int32_t>(packet.readUInt32());
        owner_.ensureItemInfo(itemEntry);
        auto* info = owner_.getItemInfo(itemEntry);
        std::string rawName = info && !info->name.empty() ? info->name : ("Item #" + std::to_string(itemEntry));
        if (bidRandProp != 0) {
            std::string suffix = owner_.getRandomPropertyName(bidRandProp);
            if (!suffix.empty()) rawName += " " + suffix;
        }
        uint32_t bidQuality = info ? info->quality : 1u;
        owner_.addSystemChatMessage("You have been outbid on " + buildItemLink(itemEntry, bidQuality, rawName) + ".");
        packet.skipAll();
    };

    table[Opcode::SMSG_AUCTION_REMOVED_NOTIFICATION] = [this](network::Packet& packet) {
        if (packet.hasRemaining(12)) {
            /*uint32_t auctionId =*/ packet.readUInt32();
            uint32_t itemEntry = packet.readUInt32();
            int32_t itemRandom = static_cast<int32_t>(packet.readUInt32());
            owner_.ensureItemInfo(itemEntry);
            auto* info = owner_.getItemInfo(itemEntry);
            std::string rawName3 = info && !info->name.empty() ? info->name : ("Item #" + std::to_string(itemEntry));
            if (itemRandom != 0) {
                std::string suffix = owner_.getRandomPropertyName(itemRandom);
                if (!suffix.empty()) rawName3 += " " + suffix;
            }
            uint32_t remQuality = info ? info->quality : 1u;
            std::string remLink = buildItemLink(itemEntry, remQuality, rawName3);
            owner_.addSystemChatMessage("Your auction of " + remLink + " has expired.");
        }
        packet.skipAll();
    };

    // ---- Equipment Sets ----
    table[Opcode::SMSG_EQUIPMENT_SET_LIST] = [this](network::Packet& packet) { handleEquipmentSetList(packet); };
    table[Opcode::SMSG_EQUIPMENT_SET_SAVED] = [this](network::Packet& packet) {
        std::string setName;
        if (packet.hasRemaining(12)) {
            uint32_t setIndex = packet.readUInt32();
            uint64_t setGuid  = packet.readUInt64();
            bool found = false;
            for (auto& es : equipmentSets_) {
                if (es.setGuid == setGuid || es.setId == setIndex) {
                    es.setGuid = setGuid;
                    setName = es.name;
                    found = true;
                    break;
                }
            }
            for (auto& info : equipmentSetInfo_) {
                if (info.setGuid == setGuid || info.setId == setIndex) {
                    info.setGuid = setGuid;
                    break;
                }
            }
            if (!found && setGuid != 0) {
                EquipmentSet newEs;
                newEs.setGuid = setGuid;
                newEs.setId   = setIndex;
                newEs.name    = pendingSaveSetName_;
                newEs.iconName = pendingSaveSetIcon_;
                for (int s = 0; s < 19; ++s)
                    newEs.itemGuids[s] = owner_.getEquipSlotGuid(s);
                equipmentSets_.push_back(std::move(newEs));
                EquipmentSetInfo newInfo;
                newInfo.setGuid = setGuid;
                newInfo.setId   = setIndex;
                newInfo.name    = pendingSaveSetName_;
                newInfo.iconName = pendingSaveSetIcon_;
                equipmentSetInfo_.push_back(std::move(newInfo));
                setName = pendingSaveSetName_;
            }
            pendingSaveSetName_.clear();
            pendingSaveSetIcon_.clear();
            LOG_INFO("SMSG_EQUIPMENT_SET_SAVED: index=", setIndex,
                     " guid=", setGuid, " name=", setName);
        }
        owner_.addSystemChatMessage(setName.empty()
            ? std::string("Equipment set saved.")
            : "Equipment set \"" + setName + "\" saved.");
    };

    table[Opcode::SMSG_EQUIPMENT_SET_USE_RESULT] = [this](network::Packet& packet) {
        if (packet.hasRemaining(1)) {
            uint8_t result = packet.readUInt8();
            if (result != 0) { owner_.addUIError("Failed to equip item set."); owner_.addSystemChatMessage("Failed to equip item set."); }
        }
    };

    // ---- Item text ----
    table[Opcode::SMSG_ITEM_TEXT_QUERY_RESPONSE] = [this](network::Packet& packet) { handleItemTextQueryResponse(packet); };

    // ---- Trade ----
    table[Opcode::SMSG_TRADE_STATUS] = [this](network::Packet& packet) { handleTradeStatus(packet); };
    table[Opcode::SMSG_TRADE_STATUS_EXTENDED] = [this](network::Packet& packet) { handleTradeStatusExtended(packet); };

    // ---- Trainer buy ----
    table[Opcode::SMSG_TRAINER_BUY_SUCCEEDED] = [this](network::Packet& p) { handleTrainerBuySucceeded(p); };
    table[Opcode::SMSG_TRAINER_BUY_FAILED] = [this](network::Packet& p) { handleTrainerBuyFailed(p); };
}

// ============================================================
// Loot
// ============================================================

void InventoryHandler::lootTarget(uint64_t targetGuid) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    currentLoot_.items.clear();
    LOG_INFO("Looting target 0x", std::hex, targetGuid, std::dec);
    auto packet = LootPacket::build(targetGuid);
    owner_.getSocket()->send(packet);
}

void InventoryHandler::lootItem(uint8_t slotIndex) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    auto packet = AutostoreLootItemPacket::build(slotIndex);
    owner_.getSocket()->send(packet);
}

void InventoryHandler::closeLoot() {
    if (!lootWindowOpen_) return;
    const uint64_t lootGuid = currentLoot_.lootGuid;
    if (owner_.getState() == WorldState::IN_WORLD && owner_.getSocket()) {
        auto packet = LootReleasePacket::build(lootGuid);
        owner_.getSocket()->send(packet);
    }
    lootWindowOpen_ = false;
    if (owner_.lootWindowCallbackRef()) owner_.lootWindowCallbackRef()(false);
    if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("LOOT_CLOSED", {});
    // Do NOT locally despawn a gather node on loot-close. Node lifetime is
    // server-authoritative: a mineral vein / herb can still hold charges for another
    // gatherer, and the server sends SMSG_DESTROY_OBJECT when it is truly depleted.
    // Predicting the despawn here removed the node after a single loot, so it couldn't
    // be shared or re-mined (it looked like one player "mined it all").
    currentLoot_ = LootResponseData{};
}

void InventoryHandler::lootMasterGive(uint8_t lootSlot, uint64_t targetGuid) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    network::Packet pkt(wireOpcode(Opcode::CMSG_LOOT_MASTER_GIVE));
    pkt.writeUInt64(currentLoot_.lootGuid);
    pkt.writeUInt8(lootSlot);
    pkt.writeUInt64(targetGuid);
    owner_.getSocket()->send(pkt);
}

void InventoryHandler::handleLootResponse(network::Packet& packet) {
    const bool wotlkLoot = isActiveExpansion("wotlk");
    if (!LootResponseParser::parse(packet, currentLoot_, wotlkLoot)) return;
    const bool hasLoot = !currentLoot_.items.empty() || currentLoot_.gold > 0;
    LOG_INFO("SMSG_LOOT_RESPONSE: guid=0x", std::hex, currentLoot_.lootGuid, std::dec,
             " items=", currentLoot_.items.size(), " gold=", currentLoot_.gold);
    auto& lastInteractedGoGuid = owner_.lastInteractedGoGuidRef();
    const bool isLastInteractedGoLoot =
        lastInteractedGoGuid != 0 && currentLoot_.lootGuid == lastInteractedGoGuid;
    if (!hasLoot && isLastInteractedGoLoot &&
        ((owner_.isCasting() && owner_.getCurrentCastSpellId() != 0) ||
         owner_.hasPendingGameObjectLootOpen(currentLoot_.lootGuid))) {
        LOG_DEBUG("Ignoring empty SMSG_LOOT_RESPONSE during pending gather/open");
        return;
    }
    lootWindowOpen_ = true;
    if (owner_.lootWindowCallbackRef()) owner_.lootWindowCallbackRef()(true);
    if (owner_.addonEventCallbackRef()) {
        owner_.addonEventCallbackRef()("LOOT_OPENED", {});
        owner_.addonEventCallbackRef()("LOOT_READY", {});
    }
    if (currentLoot_.lootGuid == lastInteractedGoGuid) {
        lastInteractedGoGuid = 0;
    }
    owner_.clearPendingGameObjectLootOpen(currentLoot_.lootGuid);
    auto& localLoot = localLootState_[currentLoot_.lootGuid];
    localLoot.data = currentLoot_;

    for (const auto& item : currentLoot_.items) {
        owner_.queryItemInfo(item.itemId, 0);
    }

    if (currentLoot_.gold > 0) {
        if (owner_.getState() == WorldState::IN_WORLD && owner_.getSocket()) {
            bool suppressFallback = false;
            auto cooldownIt = recentLootMoneyAnnounceCooldowns_.find(currentLoot_.lootGuid);
            if (cooldownIt != recentLootMoneyAnnounceCooldowns_.end() && cooldownIt->second > 0.0f) {
                suppressFallback = true;
            }
            pendingLootMoneyGuid_ = suppressFallback ? 0 : currentLoot_.lootGuid;
            pendingLootMoneyAmount_ = suppressFallback ? 0 : currentLoot_.gold;
            pendingLootMoneyNotifyTimer_ = suppressFallback ? 0.0f : 0.4f;
            auto pkt = LootMoneyPacket::build();
            owner_.getSocket()->send(pkt);
            currentLoot_.gold = 0;
        }
    }

    // Game-object quest containers are single-action pickups in normal play.
    // Autostore their returned items even when general creature auto-loot is off;
    // otherwise right-click opens server loot but never grants the objective item.
    const bool autoStoreGameObjectLoot = isLastInteractedGoLoot;
    if ((autoLoot_ || autoStoreGameObjectLoot) &&
        owner_.getState() == WorldState::IN_WORLD && owner_.getSocket() &&
        !localLoot.itemAutoLootSent) {
        for (const auto& item : currentLoot_.items) {
            LOG_INFO("Autostoring loot slot=", static_cast<int>(item.slotIndex),
                     " item=", item.itemId, " count=", item.count,
                     " fromGuid=0x", std::hex, currentLoot_.lootGuid, std::dec,
                     " gameObject=", autoStoreGameObjectLoot);
            auto pkt = AutostoreLootItemPacket::build(item.slotIndex);
            owner_.getSocket()->send(pkt);
        }
        localLoot.itemAutoLootSent = true;
    }
}

void InventoryHandler::handleLootReleaseResponse(network::Packet& packet) {
    (void)packet;
    const uint64_t lootGuid = currentLoot_.lootGuid;
    localLootState_.erase(lootGuid);
    lootWindowOpen_ = false;
    if (owner_.lootWindowCallbackRef()) owner_.lootWindowCallbackRef()(false);
    if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("LOOT_CLOSED", {});
    // Node lifetime is server-authoritative — see closeLoot(). The server despawns a
    // depleted gather node via SMSG_DESTROY_OBJECT; don't predict it here or a node the
    // server keeps (still holding charges for another gatherer) disappears locally.
    currentLoot_ = LootResponseData{};
}

void InventoryHandler::handleLootRemoved(network::Packet& packet) {
    uint8_t slotIndex = packet.readUInt8();
    for (auto it = currentLoot_.items.begin(); it != currentLoot_.items.end(); ++it) {
        if (it->slotIndex == slotIndex) {
            // SMSG_ITEM_PUSH_RESULT is the authoritative inventory receipt and
            // emits the single "Received item" notification. Slot removal only
            // updates the open loot window; announcing here duplicated chat and
            // the loot sound for the same item.
            currentLoot_.items.erase(it);
            if (owner_.addonEventCallbackRef())
                owner_.addonEventCallbackRef()("LOOT_SLOT_CLEARED", {std::to_string(slotIndex + 1)});
            break;
        }
    }
}

// ============================================================
// Loot Roll
// ============================================================

void InventoryHandler::sendLootRoll(uint64_t objectGuid, uint32_t slot, uint8_t rollType) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    network::Packet pkt(wireOpcode(Opcode::CMSG_LOOT_ROLL));
    pkt.writeUInt64(objectGuid);
    pkt.writeUInt32(slot);
    pkt.writeUInt8(rollType);
    owner_.getSocket()->send(pkt);
    // Once we've sent any choice (pass/need/greed/disenchant), close the dialog.
    pendingLootRollActive_ = false;
}

void InventoryHandler::handleLootRoll(network::Packet& packet) {
    // objectGuid(8) + lootSlot(4) + playerGuid(8) + itemId(4) + itemRandSuffix(4) +
    // itemRandProp(4) + rollNumber(1) + rollType(1) + autoPass(1)
    if (!packet.hasRemaining(35)) return;
    uint64_t objectGuid = packet.readUInt64();
    uint32_t lootSlot   = packet.readUInt32();
    uint64_t playerGuid = packet.readUInt64();
    /*uint32_t itemId     =*/ packet.readUInt32();
    /*uint32_t randSuffix =*/ packet.readUInt32();
    (void)packet.readUInt32(); // random property
    uint8_t rollNumber  = packet.readUInt8();
    uint8_t rollType    = packet.readUInt8();
    /*uint8_t autoPass   =*/ packet.readUInt8();

    // Resolve player name
    std::string playerName;
    auto nit = owner_.getPlayerNameCache().find(playerGuid);
    if (nit != owner_.getPlayerNameCache().end()) playerName = nit->second;
    if (playerName.empty()) playerName = "Player";

    if (pendingLootRollActive_ &&
        pendingLootRoll_.objectGuid == objectGuid &&
        pendingLootRoll_.slot == lootSlot) {
        LootRollEntry::PlayerRollResult result;
        result.playerName = playerName;
        result.rollNum = rollNumber;
        result.rollType = rollType;
        pendingLootRoll_.playerRolls.push_back(result);
    }

    // RollVote enum: 0=Pass, 1=Need, 2=Greed, 3=Disenchant.
    const char* typeStr = "passed on";
    if (rollType == 1) typeStr = "rolled Need";
    else if (rollType == 2) typeStr = "rolled Greed";
    else if (rollType == 3) typeStr = "rolled Disenchant";
    if (rollType >= 1 && rollType <= 3) {
        owner_.addSystemChatMessage(playerName + " " + typeStr + " - " + std::to_string(rollNumber));
    } else {
        owner_.addSystemChatMessage(playerName + " passed.");
    }
}

void InventoryHandler::handleLootRollWon(network::Packet& packet) {
    // objectGuid(8) + lootSlot(4) + itemId(4) + itemSuffix(4) + itemProp(4) + playerGuid(8) + rollNumber(1) + rollType(1)
    if (!packet.hasRemaining(34)) return;
    /*uint64_t objectGuid =*/ packet.readUInt64();
    /*uint32_t lootSlot   =*/ packet.readUInt32();
    uint32_t itemId     = packet.readUInt32();
    /*uint32_t randSuffix =*/ packet.readUInt32();
    int32_t wonRandProp = static_cast<int32_t>(packet.readUInt32());
    uint64_t winnerGuid = packet.readUInt64();
    uint8_t rollNumber  = packet.readUInt8();
    uint8_t rollType    = packet.readUInt8();

    std::string winnerName;
    auto nit = owner_.getPlayerNameCache().find(winnerGuid);
    if (nit != owner_.getPlayerNameCache().end()) winnerName = nit->second;
    if (winnerName.empty()) winnerName = "Player";

    owner_.ensureItemInfo(itemId);
    auto* info = owner_.getItemInfo(itemId);
    std::string itemName = (info && !info->name.empty()) ? info->name : ("Item #" + std::to_string(itemId));
    if (wonRandProp != 0) {
        std::string suffix = owner_.getRandomPropertyName(wonRandProp);
        if (!suffix.empty()) itemName += " " + suffix;
    }
    uint32_t wonQuality = info ? info->quality : 1u;
    std::string link = buildItemLink(itemId, wonQuality, itemName);

    // RollVote enum: 1=Need, 2=Greed, 3=Disenchant (0=Pass cannot win).
    const char* typeStr = "Need";
    if (rollType == 2) typeStr = "Greed";
    else if (rollType == 3) typeStr = "Disenchant";

    owner_.addSystemChatMessage(winnerName + " won " + link + " (" + typeStr + " - " + std::to_string(rollNumber) + ")");
    pendingLootRollActive_ = false;
}

// ============================================================
// Vendor
// ============================================================

void InventoryHandler::openVendor(uint64_t npcGuid) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    auto packet = ListInventoryPacket::build(npcGuid);
    owner_.getSocket()->send(packet);
}

void InventoryHandler::closeVendor() {
    bool wasOpen = vendorWindowOpen_;
    vendorWindowOpen_ = false;
    currentVendorItems_ = ListInventoryData{};
    // Keep buybackItems_ and pendingSellToBuyback_: buyback slots live on the
    // player server-side (wire slots 74-85) and persist across vendor windows,
    // so the local mirror must too — clearing here made the buyback list
    // vanish when the vendor was reopened. The mirror resets on world entry.
    pendingBuybackSlot_ = -1;
    pendingBuybackWireSlot_ = 0;
    pendingBuyItemId_ = 0;
    pendingBuyItemSlot_ = 0;
    if (wasOpen && owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("MERCHANT_CLOSED", {});
}

void InventoryHandler::clearBuybackState() {
    buybackItems_.clear();
    pendingSellToBuyback_.clear();
    buybackSlotGuids_.fill(0);
    pendingBuybackSlot_ = -1;
    pendingBuybackWireSlot_ = 0;
}

void InventoryHandler::reconcileBuybackSlots() {
    for (auto it = buybackItems_.begin(); it != buybackItems_.end();) {
        const auto slot = std::find(buybackSlotGuids_.begin(),
                                    buybackSlotGuids_.end(), it->itemGuid);
        if (slot != buybackSlotGuids_.end()) {
            it->wireSlot = kBuybackWireSlotStart +
                static_cast<uint32_t>(std::distance(buybackSlotGuids_.begin(), slot));
            pendingSellToBuyback_.erase(it->itemGuid);
            ++it;
        } else if (it->wireSlot != 0) {
            // A previously confirmed item disappeared from the server-owned
            // buyback fields (purchase or ring replacement).
            it = buybackItems_.erase(it);
        } else {
            // SMSG_SELL_ITEM and the player-field delta may arrive in either
            // order. Keep the just-sold entry hidden from clicks until mapped.
            ++it;
        }
    }

    // A ring slot can only identify one entry. Remove any stale duplicate that
    // may remain after a replacement update.
    for (uint32_t wire = kBuybackWireSlotStart; wire < kBuybackWireSlotEnd; ++wire) {
        bool kept = false;
        for (auto it = buybackItems_.begin(); it != buybackItems_.end();) {
            if (it->wireSlot != wire) {
                ++it;
                continue;
            }
            if (!kept) {
                kept = true;
                ++it;
            } else {
                it = buybackItems_.erase(it);
            }
        }
    }
}

void InventoryHandler::buyItem(uint64_t vendorGuid, uint32_t itemId, uint32_t slot, uint32_t count) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    LOG_INFO("Buy request: vendorGuid=0x", std::hex, vendorGuid, std::dec,
             " itemId=", itemId, " slot=", slot, " count=", count,
             " wire=0x", std::hex, wireOpcode(Opcode::CMSG_BUY_ITEM), std::dec);
    pendingBuyItemId_ = itemId;
    pendingBuyItemSlot_ = slot;
    network::Packet packet(wireOpcode(Opcode::CMSG_BUY_ITEM));
    packet.writeUInt64(vendorGuid);
    packet.writeUInt32(itemId);
    packet.writeUInt32(slot);
    packet.writeUInt32(count);
    const bool isWotLk = isActiveExpansion("wotlk");
    if (isWotLk) {
        packet.writeUInt8(0);
    }
    owner_.getSocket()->send(packet);
}

void InventoryHandler::sellItem(uint64_t vendorGuid, uint64_t itemGuid, uint32_t count) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    LOG_INFO("Sell request: vendorGuid=0x", std::hex, vendorGuid,
             " itemGuid=0x", itemGuid, std::dec,
             " count=", count, " wire=0x", std::hex, wireOpcode(Opcode::CMSG_SELL_ITEM), std::dec);
    auto packet = SellItemPacket::build(vendorGuid, itemGuid, count);
    owner_.getSocket()->send(packet);
}

void InventoryHandler::sellItemBySlot(int backpackIndex) {
    if (backpackIndex < 0 || backpackIndex >= owner_.inventoryRef().getBackpackSize()) return;
    const auto& slot = owner_.inventoryRef().getBackpackSlot(backpackIndex);
    if (slot.empty()) return;

    uint32_t sellPrice = slot.item.sellPrice;
    if (sellPrice == 0) {
        if (auto* info = owner_.getItemInfo(slot.item.itemId); info && info->valid) {
            sellPrice = info->sellPrice;
        }
    }
    if (sellPrice == 0) {
        owner_.addSystemChatMessage("Cannot sell: this item has no vendor value.");
        return;
    }

    uint64_t itemGuid = slot.item.guid;
    if (itemGuid == 0) itemGuid = owner_.backpackSlotGuidsRef()[backpackIndex];
    if (itemGuid == 0) itemGuid = owner_.resolveOnlineItemGuid(slot.item.itemId);
    LOG_DEBUG("sellItemBySlot: slot=", backpackIndex,
              " item=", slot.item.name,
              " itemGuid=0x", std::hex, itemGuid, std::dec,
              " vendorGuid=0x", std::hex, currentVendorItems_.vendorGuid, std::dec);
    if (itemGuid != 0 && currentVendorItems_.vendorGuid != 0) {
        BuybackItem sold;
        sold.itemGuid = itemGuid;
        sold.item = slot.item;
        sold.count = 1;
        pendingSellToBuyback_[itemGuid] = sold;
        buybackItems_.push_back(sold);
        sellItem(currentVendorItems_.vendorGuid, itemGuid, 1);
    } else if (itemGuid == 0) {
        owner_.addSystemChatMessage("Cannot sell: item not found in inventory.");
        LOG_WARNING("Sell failed: missing item GUID for slot ", backpackIndex);
    } else {
        owner_.addSystemChatMessage("Cannot sell: no vendor.");
    }
}

void InventoryHandler::sellItemInBag(int bagIndex, int slotIndex) {
    if (bagIndex < 0 || bagIndex >= owner_.inventoryRef().NUM_BAG_SLOTS) return;
    if (slotIndex < 0 || slotIndex >= owner_.inventoryRef().getBagSize(bagIndex)) return;
    const auto& slot = owner_.inventoryRef().getBagSlot(bagIndex, slotIndex);
    if (slot.empty()) return;

    uint32_t sellPrice = slot.item.sellPrice;
    if (sellPrice == 0) {
        if (auto* info = owner_.getItemInfo(slot.item.itemId); info && info->valid) {
            sellPrice = info->sellPrice;
        }
    }
    if (sellPrice == 0) {
        owner_.addSystemChatMessage("Cannot sell: this item has no vendor value.");
        return;
    }

    uint64_t itemGuid = slot.item.guid;
    uint64_t bagGuid = owner_.equipSlotGuidsRef()[Inventory::FIRST_BAG_EQUIP_SLOT + bagIndex];
    if (itemGuid == 0 && bagGuid != 0) {
        auto it = owner_.containerContentsRef().find(bagGuid);
        if (it != owner_.containerContentsRef().end() && slotIndex < static_cast<int>(it->second.numSlots)) {
            itemGuid = it->second.slotGuids[slotIndex];
        }
    }
    if (itemGuid == 0) {
        itemGuid = owner_.resolveOnlineItemGuid(slot.item.itemId);
    }

    if (itemGuid != 0 && currentVendorItems_.vendorGuid != 0) {
        BuybackItem sold;
        sold.itemGuid = itemGuid;
        sold.item = slot.item;
        sold.count = 1;
        pendingSellToBuyback_[itemGuid] = sold;
        buybackItems_.push_back(sold);
        sellItem(currentVendorItems_.vendorGuid, itemGuid, 1);
    } else if (itemGuid == 0) {
        owner_.addSystemChatMessage("Cannot sell: item not found.");
    } else {
        owner_.addSystemChatMessage("Cannot sell: no vendor.");
    }
}

void InventoryHandler::buyBackItem(uint32_t buybackSlot) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket() || currentVendorItems_.vendorGuid == 0) return;
    if (buybackSlot >= buybackItems_.size()) return;
    const uint32_t wireSlot = buybackItems_[buybackSlot].wireSlot;
    if (wireSlot < kBuybackWireSlotStart || wireSlot >= kBuybackWireSlotEnd) {
        LOG_WARNING("Buyback request rejected: row ", buybackSlot,
                    " has invalid wire slot ", wireSlot);
        return;
    }
    pendingBuyItemId_ = 0;
    pendingBuyItemSlot_ = 0;
    LOG_INFO("Buyback request: vendorGuid=0x", std::hex, currentVendorItems_.vendorGuid,
             std::dec, " uiSlot=", buybackSlot, " wireSlot=", wireSlot);
    pendingBuybackSlot_ = static_cast<int>(buybackSlot);
    pendingBuybackWireSlot_ = wireSlot;
    // Use the expansion-agnostic packet builder so the opcode resolves from
    // the active expansion's JSON mapping rather than a hardcoded WotLK value.
    owner_.getSocket()->send(BuybackItemPacket::build(currentVendorItems_.vendorGuid, wireSlot));
}

void InventoryHandler::repairItem(uint64_t vendorGuid, uint64_t itemGuid) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;

    uint32_t cost = estimateItemRepairCost(itemGuid);
    if (cost > 0 && owner_.getMoneyCopper() < cost) {
        owner_.addUIError("Not enough money");
        return;
    }

    network::Packet packet(wireOpcode(Opcode::CMSG_REPAIR_ITEM));
    packet.writeUInt64(vendorGuid);
    packet.writeUInt64(itemGuid);
    if (!isClassicLikeExpansion()) packet.writeUInt8(0);
    owner_.getSocket()->send(packet);

    // Only do optimistic update if we verified the player can afford it
    if (cost > 0) {
        owner_.playerMoneyCopperRef() -= cost;
        auto it = owner_.onlineItemsRef().find(itemGuid);
        if (it != owner_.onlineItemsRef().end()) {
            it->second.curDurability = it->second.maxDurability;
            rebuildOnlineInventory();
        }
    }
}

void InventoryHandler::repairAll(uint64_t vendorGuid, bool useGuildBank) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;

    uint32_t totalCost = estimateRepairAllCost();

    if (!useGuildBank && totalCost > 0 && owner_.getMoneyCopper() < totalCost) {
        owner_.addUIError("Not enough money");
        return;
    }

    network::Packet packet(wireOpcode(Opcode::CMSG_REPAIR_ITEM));
    packet.writeUInt64(vendorGuid);
    packet.writeUInt64(0);
    if (!isClassicLikeExpansion()) packet.writeUInt8(useGuildBank ? 1 : 0);
    owner_.getSocket()->send(packet);

    // Only do optimistic update for the player-funded path: we verified the
    // player has the gold, so server acceptance is essentially guaranteed.
    //
    // Guild-bank repair (useGuildBank=true) cannot be confirmed client-side:
    // the server silently rejects when the player has no guild, no
    // GUILD_BANK_RIGHT_REPAIR permission, or the guild bank lacks funds —
    // in all those cases Player::DurabilityRepair returns early WITHOUT
    // setting durability and WITHOUT sending an UPDATE_OBJECT, so any
    // optimistic durability bump would persist on screen until relog
    // (when the server's actual state reloads). Wait for the server's
    // UPDATE_OBJECT to confirm instead.
    if (totalCost > 0 && !useGuildBank) {
        owner_.playerMoneyCopperRef() -= totalCost;
        for (auto& [guid, info] : owner_.onlineItemsRef()) {
            if (info.maxDurability > 0 && info.curDurability < info.maxDurability) {
                info.curDurability = info.maxDurability;
            }
        }
        rebuildOnlineInventory();
    }
}

void InventoryHandler::autoEquipItemBySlot(int backpackIndex) {
    if (backpackIndex < 0 || backpackIndex >= owner_.inventoryRef().getBackpackSize()) return;
    const auto& slot = owner_.inventoryRef().getBackpackSlot(backpackIndex);
    if (slot.empty()) return;

    if (owner_.getState() == WorldState::IN_WORLD && owner_.getSocket()) {
        auto packet = AutoEquipItemPacket::build(0xFF, static_cast<uint8_t>(Inventory::NUM_EQUIP_SLOTS + backpackIndex));
        owner_.getSocket()->send(packet);
    }
}

void InventoryHandler::autoEquipItemInBag(int bagIndex, int slotIndex) {
    if (bagIndex < 0 || bagIndex >= owner_.inventoryRef().NUM_BAG_SLOTS) return;
    if (slotIndex < 0 || slotIndex >= owner_.inventoryRef().getBagSize(bagIndex)) return;

    if (owner_.getState() == WorldState::IN_WORLD && owner_.getSocket()) {
        auto packet = AutoEquipItemPacket::build(
            static_cast<uint8_t>(Inventory::FIRST_BAG_EQUIP_SLOT + bagIndex), static_cast<uint8_t>(slotIndex));
        owner_.getSocket()->send(packet);
    }
}

// Dispatches CMSG_USE_ITEM for an item already located at (wowBag, wowSlot).
// Spells that enchant another item (sharpening stones, weightstones, weapon oils)
// cannot be sent immediately: they need the target item's GUID, so the use is
// parked and completed by completeItemUseOnItem() once the player picks a target.
void InventoryHandler::dispatchUseItem(uint8_t wowBag, uint8_t wowSlot, uint64_t itemGuid,
                                       const ItemDef& item) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    if (owner_.isRestoring()) owner_.cancelCast();
    if (itemGuid == 0) {
        LOG_WARNING("useItem: itemGuid=0 for item='", item.name, "' entry=", item.itemId,
                    " — cannot use");
        owner_.addSystemChatMessage("Cannot use that item right now.");
        return;
    }

    const auto* itemInfo = owner_.getItemInfo(item.itemId);
    uint32_t useSpellId = 0;
    if (itemInfo) {
        for (const auto& sp : itemInfo->spells) {
            if (sp.spellId != 0 && (sp.spellTrigger == 0 || sp.spellTrigger == 5)) {
                useSpellId = sp.spellId;
                break;
            }
        }
    }
    LOG_DEBUG("useItem: bag=", (int)wowBag, " slot=", (int)wowSlot, " entry=", item.itemId,
              " spellId=", useSpellId);

    // Pre-WotLK mounts are items, so the same rule applies to them: using the
    // one you are riding dismounts you rather than re-summoning it.
    if (useSpellId != 0 && owner_.isMounted() &&
        useSpellId == owner_.getMountAuraSpellId()) {
        owner_.dismount();
        LOG_INFO("Dismount via mount item: entry=", item.itemId, " spell=", useSpellId);
        return;
    }
    if (useSpellId != 0 && !owner_.isMounted() && owner_.getSpellHandler()) {
        owner_.getSpellHandler()->noteGroundCastSpell(useSpellId);
    }

    if (useSpellId != 0 &&
        (owner_.getSpellTargetFlags(useSpellId) & kSpellTargetFlagItem) != 0) {
        pendingItemTarget_ = PendingItemTarget{wowBag, wowSlot, itemGuid, useSpellId,
                                               item.itemId, item.name};
        owner_.addSystemChatMessage("Choose an item to apply " + item.name + " to.");
        return;
    }

    if (isBandageItem(itemInfo)) synchronizeStationaryBandageCast(owner_);
    sendUseItem(wowBag, wowSlot, itemGuid, useSpellId,
                targetGuidForUseItem(owner_, itemInfo), 0);
}

void InventoryHandler::sendUseItem(uint8_t wowBag, uint8_t wowSlot, uint64_t itemGuid,
                                   uint32_t spellId, uint64_t targetGuid, uint64_t itemTargetGuid) {
    auto packet = owner_.getPacketParsers()
        ? owner_.getPacketParsers()->buildUseItem(wowBag, wowSlot, itemGuid, spellId,
                                                  targetGuid, itemTargetGuid)
        : UseItemPacket::build(wowBag, wowSlot, itemGuid, spellId, targetGuid, itemTargetGuid);
    LOG_INFO("Sending CMSG_USE_ITEM: bag=", (int)wowBag, " slot=", (int)wowSlot,
             " spell=", spellId, " itemTarget=0x", std::hex, itemTargetGuid, std::dec,
             " packetSize=", packet.getSize());
    owner_.getSocket()->send(packet);
}

bool InventoryHandler::isAwaitingItemTarget() const {
    if (!pendingItemTarget_) return false;
    // Leaving the world abandons the pending use — the slot and GUID would be
    // stale for the next character.
    if (owner_.getState() != WorldState::IN_WORLD) {
        pendingItemTarget_.reset();
        return false;
    }
    return true;
}

uint32_t InventoryHandler::getPendingItemTargetSourceItemId() const {
    return pendingItemTarget_ ? pendingItemTarget_->itemId : 0;
}

void InventoryHandler::cancelItemTargeting() {
    pendingItemTarget_.reset();
}

void InventoryHandler::beginSpellItemTargeting(uint32_t spellId, const std::string& spellName) {
    PendingItemTarget pending;
    pending.spellId = spellId;
    pending.itemName = spellName;
    pending.fromSpell = true;
    pendingItemTarget_ = pending;
    owner_.addSystemChatMessage("Choose an item to use " + spellName + " on.");
}

void InventoryHandler::completeItemUseOnItem(uint64_t targetItemGuid) {
    if (!isAwaitingItemTarget()) return;
    const PendingItemTarget pending = *pendingItemTarget_;
    pendingItemTarget_.reset();

    if (targetItemGuid == 0 || !owner_.getSocket()) {
        owner_.addSystemChatMessage("That is not a valid target.");
        return;
    }

    if (pending.fromSpell) {
        auto packet = owner_.getPacketParsers()
            ? owner_.getPacketParsers()->buildCastSpellOnItem(pending.spellId, targetItemGuid)
            : CastSpellPacket::buildItemTarget(pending.spellId, targetItemGuid, 0);
        owner_.getSocket()->send(packet);
        LOG_INFO("Casting ", pending.itemName, " (spell ", pending.spellId, ") on item 0x",
                 std::hex, targetItemGuid, std::dec);
        return;
    }
    // Applying to itself is never valid and the server would silently drop it.
    if (targetItemGuid == pending.itemGuid) {
        owner_.addSystemChatMessage("You cannot apply " + pending.itemName + " to itself.");
        return;
    }

    sendUseItem(pending.bag, pending.slot, pending.itemGuid, pending.spellId, 0, targetItemGuid);
}

void InventoryHandler::useItemBySlot(int backpackIndex) {
    if (backpackIndex < 0 || backpackIndex >= owner_.inventoryRef().getBackpackSize()) return;
    const auto& slot = owner_.inventoryRef().getBackpackSlot(backpackIndex);
    if (slot.empty()) return;

    uint64_t itemGuid = owner_.backpackSlotGuidsRef()[backpackIndex];
    if (itemGuid == 0) {
        itemGuid = owner_.resolveOnlineItemGuid(slot.item.itemId);
    }

    dispatchUseItem(0xFF, static_cast<uint8_t>(Inventory::NUM_EQUIP_SLOTS + backpackIndex),
                    itemGuid, slot.item);
}

void InventoryHandler::useItemInBag(int bagIndex, int slotIndex) {
    if (bagIndex < 0 || bagIndex >= owner_.inventoryRef().NUM_BAG_SLOTS) return;
    if (slotIndex < 0 || slotIndex >= owner_.inventoryRef().getBagSize(bagIndex)) return;
    const auto& slot = owner_.inventoryRef().getBagSlot(bagIndex, slotIndex);
    if (slot.empty()) return;

    uint64_t itemGuid = 0;
    uint64_t bagGuid = owner_.equipSlotGuidsRef()[Inventory::FIRST_BAG_EQUIP_SLOT + bagIndex];
    if (bagGuid != 0) {
        auto it = owner_.containerContentsRef().find(bagGuid);
        if (it != owner_.containerContentsRef().end() && slotIndex < static_cast<int>(it->second.numSlots)) {
            itemGuid = it->second.slotGuids[slotIndex];
        }
    }
    if (itemGuid == 0) {
        itemGuid = owner_.resolveOnlineItemGuid(slot.item.itemId);
    }

    LOG_INFO("useItemInBag: bag=", bagIndex, " slot=", slotIndex, " itemId=", slot.item.itemId,
             " itemGuid=0x", std::hex, itemGuid, std::dec);

    dispatchUseItem(static_cast<uint8_t>(Inventory::FIRST_BAG_EQUIP_SLOT + bagIndex),
                    static_cast<uint8_t>(slotIndex), itemGuid, slot.item);
}

void InventoryHandler::openItemBySlot(int backpackIndex) {
    if (backpackIndex < 0 || backpackIndex >= owner_.inventoryRef().getBackpackSize()) return;
    if (owner_.inventoryRef().getBackpackSlot(backpackIndex).empty()) return;
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    auto packet = OpenItemPacket::build(0xFF, static_cast<uint8_t>(Inventory::NUM_EQUIP_SLOTS + backpackIndex));
    LOG_INFO("openItemBySlot: CMSG_OPEN_ITEM bag=0xFF slot=", (Inventory::NUM_EQUIP_SLOTS + backpackIndex));
    owner_.getSocket()->send(packet);
}

void InventoryHandler::openItemInBag(int bagIndex, int slotIndex) {
    if (bagIndex < 0 || bagIndex >= owner_.inventoryRef().NUM_BAG_SLOTS) return;
    if (slotIndex < 0 || slotIndex >= owner_.inventoryRef().getBagSize(bagIndex)) return;
    if (owner_.inventoryRef().getBagSlot(bagIndex, slotIndex).empty()) return;
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    uint8_t wowBag = static_cast<uint8_t>(Inventory::FIRST_BAG_EQUIP_SLOT + bagIndex);
    auto packet = OpenItemPacket::build(wowBag, static_cast<uint8_t>(slotIndex));
    LOG_INFO("openItemInBag: CMSG_OPEN_ITEM bag=", (int)wowBag, " slot=", slotIndex);
    owner_.getSocket()->send(packet);
}

void InventoryHandler::readItemBySlot(int backpackIndex) {
    if (backpackIndex < 0 || backpackIndex >= owner_.inventoryRef().getBackpackSize()) return;
    const auto& slot = owner_.inventoryRef().getBackpackSlot(backpackIndex);
    if (slot.empty()) return;
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;

    const auto* info = owner_.getItemInfo(slot.item.itemId);
    uint32_t pageTextId = info ? info->pageTextId : slot.item.pageTextId;
    if (pageTextId == 0) return;

    uint64_t itemGuid = owner_.backpackSlotGuidsRef()[backpackIndex];
    if (itemGuid == 0) itemGuid = owner_.resolveOnlineItemGuid(slot.item.itemId);

    owner_.bookPagesRef().clear();
    auto readPacket = ReadItemPacket::build(0xFF, static_cast<uint8_t>(Inventory::NUM_EQUIP_SLOTS + backpackIndex));
    owner_.getSocket()->send(readPacket);
    auto pagePacket = PageTextQueryPacket::build(pageTextId, itemGuid != 0 ? itemGuid : owner_.getPlayerGuid());
    owner_.getSocket()->send(pagePacket);
    LOG_INFO("readItemBySlot: CMSG_READ_ITEM pageTextId=", pageTextId,
             " bag=0xFF slot=", (Inventory::NUM_EQUIP_SLOTS + backpackIndex));
}

void InventoryHandler::readItemInBag(int bagIndex, int slotIndex) {
    if (bagIndex < 0 || bagIndex >= owner_.inventoryRef().NUM_BAG_SLOTS) return;
    if (slotIndex < 0 || slotIndex >= owner_.inventoryRef().getBagSize(bagIndex)) return;
    const auto& slot = owner_.inventoryRef().getBagSlot(bagIndex, slotIndex);
    if (slot.empty()) return;
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;

    const auto* info = owner_.getItemInfo(slot.item.itemId);
    uint32_t pageTextId = info ? info->pageTextId : slot.item.pageTextId;
    if (pageTextId == 0) return;

    uint64_t itemGuid = 0;
    uint64_t bagGuid = owner_.equipSlotGuidsRef()[Inventory::FIRST_BAG_EQUIP_SLOT + bagIndex];
    if (bagGuid != 0) {
        auto it = owner_.containerContentsRef().find(bagGuid);
        if (it != owner_.containerContentsRef().end() && slotIndex < static_cast<int>(it->second.numSlots)) {
            itemGuid = it->second.slotGuids[slotIndex];
        }
    }
    if (itemGuid == 0) itemGuid = owner_.resolveOnlineItemGuid(slot.item.itemId);

    uint8_t wowBag = static_cast<uint8_t>(Inventory::FIRST_BAG_EQUIP_SLOT + bagIndex);
    owner_.bookPagesRef().clear();
    auto readPacket = ReadItemPacket::build(wowBag, static_cast<uint8_t>(slotIndex));
    owner_.getSocket()->send(readPacket);
    auto pagePacket = PageTextQueryPacket::build(pageTextId, itemGuid != 0 ? itemGuid : owner_.getPlayerGuid());
    owner_.getSocket()->send(pagePacket);
    LOG_INFO("readItemInBag: CMSG_READ_ITEM pageTextId=", pageTextId,
             " bag=", static_cast<int>(wowBag), " slot=", slotIndex);
}

void InventoryHandler::destroyItem(uint8_t bag, uint8_t slot, uint8_t count) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    // Zero means the whole stack, and this used to turn it into one.
    // HandleDestroyItemOpcode branches on exactly this: a count destroys that
    // many, no count destroys the slot. Coercing it away left no way to say
    // "all of them" — which is what the confirmation prompt is asking about,
    // and the only thing a stack larger than 255 could be told to do.
    constexpr uint16_t kCmsgDestroyItem = 0x111;
    network::Packet packet(kCmsgDestroyItem);
    packet.writeUInt8(bag);
    packet.writeUInt8(slot);
    packet.writeUInt32(static_cast<uint32_t>(count));
    LOG_DEBUG("Destroy item request: bag=", (int)bag, " slot=", (int)slot,
              " count=", (int)count, " wire=0x", std::hex, kCmsgDestroyItem, std::dec);
    owner_.getSocket()->send(packet);
}

void InventoryHandler::splitItem(uint8_t srcBag, uint8_t srcSlot, uint8_t count) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    if (count == 0) return;

    int freeBp = owner_.inventoryRef().findFreeBackpackSlot();
    if (freeBp >= 0) {
        uint8_t dstBag = 0xFF;
        uint8_t dstSlot = static_cast<uint8_t>(Inventory::NUM_EQUIP_SLOTS + freeBp);
        LOG_INFO("splitItem: src(bag=", (int)srcBag, " slot=", (int)srcSlot,
                 ") count=", (int)count, " -> dst(bag=0xFF slot=", (int)dstSlot, ")");
        auto packet = SplitItemPacket::build(srcBag, srcSlot, dstBag, dstSlot, count);
        owner_.getSocket()->send(packet);
        return;
    }
    for (int b = 0; b < owner_.inventoryRef().NUM_BAG_SLOTS; b++) {
        int bagSize = owner_.inventoryRef().getBagSize(b);
        for (int s = 0; s < bagSize; s++) {
            if (owner_.inventoryRef().getBagSlot(b, s).empty()) {
                uint8_t dstBag = static_cast<uint8_t>(Inventory::FIRST_BAG_EQUIP_SLOT + b);
                uint8_t dstSlot = static_cast<uint8_t>(s);
                LOG_INFO("splitItem: src(bag=", (int)srcBag, " slot=", (int)srcSlot,
                         ") count=", (int)count, " -> dst(bag=", (int)dstBag,
                         " slot=", (int)dstSlot, ")");
                auto packet = SplitItemPacket::build(srcBag, srcSlot, dstBag, dstSlot, count);
                owner_.getSocket()->send(packet);
                return;
            }
        }
    }
    owner_.addSystemChatMessage("Cannot split: no free inventory slots.");
}

void InventoryHandler::swapContainerItems(uint8_t srcBag, uint8_t srcSlot, uint8_t dstBag, uint8_t dstSlot) {
    if (!owner_.getSocket() || !owner_.getSocket()->isConnected()) return;
    LOG_INFO("swapContainerItems: src(bag=", (int)srcBag, " slot=", (int)srcSlot,
             ") -> dst(bag=", (int)dstBag, " slot=", (int)dstSlot, ")");
    auto packet = SwapItemPacket::build(dstBag, dstSlot, srcBag, srcSlot);
    owner_.getSocket()->send(packet);
}

void InventoryHandler::swapBagSlots(int srcBagIndex, int dstBagIndex) {
    if (srcBagIndex < 0 || srcBagIndex > 3 || dstBagIndex < 0 || dstBagIndex > 3) return;
    if (srcBagIndex == dstBagIndex) return;

    auto srcEquip = static_cast<game::EquipSlot>(static_cast<int>(game::EquipSlot::BAG1) + srcBagIndex);
    auto dstEquip = static_cast<game::EquipSlot>(static_cast<int>(game::EquipSlot::BAG1) + dstBagIndex);
    auto srcItem = owner_.inventoryRef().getEquipSlot(srcEquip).item;
    auto dstItem = owner_.inventoryRef().getEquipSlot(dstEquip).item;
    owner_.inventoryRef().setEquipSlot(srcEquip, dstItem);
    owner_.inventoryRef().setEquipSlot(dstEquip, srcItem);
    owner_.inventoryRef().swapBagContents(srcBagIndex, dstBagIndex);

    if (owner_.getSocket() && owner_.getSocket()->isConnected()) {
        uint8_t srcSlot = static_cast<uint8_t>(Inventory::FIRST_BAG_EQUIP_SLOT + srcBagIndex);
        uint8_t dstSlot = static_cast<uint8_t>(Inventory::FIRST_BAG_EQUIP_SLOT + dstBagIndex);
        LOG_INFO("swapBagSlots: bag ", srcBagIndex, " (slot ", (int)srcSlot,
                 ") <-> bag ", dstBagIndex, " (slot ", (int)dstSlot, ")");
        auto packet = SwapItemPacket::build(255, dstSlot, 255, srcSlot);
        owner_.getSocket()->send(packet);
    }
}

void InventoryHandler::unequipToBackpack(EquipSlot equipSlot) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;

    int freeSlot = owner_.inventoryRef().findFreeBackpackSlot();
    if (freeSlot < 0) {
        owner_.addSystemChatMessage("Cannot unequip: no free backpack slots.");
        return;
    }

    uint8_t srcBag = 0xFF;
    uint8_t srcSlot = static_cast<uint8_t>(equipSlot);
    uint8_t dstBag = 0xFF;
    uint8_t dstSlot = static_cast<uint8_t>(Inventory::NUM_EQUIP_SLOTS + freeSlot);

    LOG_INFO("UnequipToBackpack: equipSlot=", (int)srcSlot,
             " -> backpackIndex=", freeSlot, " (dstSlot=", (int)dstSlot, ")");

    auto packet = SwapItemPacket::build(dstBag, dstSlot, srcBag, srcSlot);
    owner_.getSocket()->send(packet);
}

void InventoryHandler::useItemById(uint32_t itemId) {
    if (itemId == 0) return;
    LOG_DEBUG("useItemById: searching for itemId=", itemId);
    for (int i = 0; i < owner_.inventoryRef().getBackpackSize(); i++) {
        const auto& slot = owner_.inventoryRef().getBackpackSlot(i);
        if (!slot.empty() && slot.item.itemId == itemId) {
            LOG_DEBUG("useItemById: found itemId=", itemId, " at backpack slot ", i);
            useItemBySlot(i);
            return;
        }
    }
    for (int bag = 0; bag < owner_.inventoryRef().NUM_BAG_SLOTS; bag++) {
        int bagSize = owner_.inventoryRef().getBagSize(bag);
        for (int slot = 0; slot < bagSize; slot++) {
            const auto& bagSlot = owner_.inventoryRef().getBagSlot(bag, slot);
            if (!bagSlot.empty() && bagSlot.item.itemId == itemId) {
                LOG_DEBUG("useItemById: found itemId=", itemId, " in bag ", bag, " slot ", slot);
                useItemInBag(bag, slot);
                return;
            }
        }
    }
    LOG_WARNING("useItemById: itemId=", itemId, " not found in inventory");
}

void InventoryHandler::handleListInventory(network::Packet& packet) {
    if (!ListInventoryParser::parse(packet, currentVendorItems_)) return;

    // Detect repair capability from NPC flags (covers direct vendors without gossip).
    // UNIT_NPC_FLAG_REPAIR = 0x1000.
    if (!currentVendorItems_.canRepair && currentVendorItems_.vendorGuid != 0) {
        if (auto* unit = owner_.getUnitByGuid(currentVendorItems_.vendorGuid)) {
            if (unit->getNpcFlags() & 0x1000) {
                currentVendorItems_.canRepair = true;
            }
        }
    }
    vendorWindowOpen_ = true;
    owner_.closeGossip();
    if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("MERCHANT_SHOW", {});

    // Auto-sell grey items
    if (autoSellGrey_ && currentVendorItems_.vendorGuid != 0 && owner_.getState() == WorldState::IN_WORLD && owner_.getSocket()) {
        int itemsSold = 0;
        uint32_t totalSellPrice = 0;
        for (int i = 0; i < owner_.inventoryRef().getBackpackSize(); ++i) {
            const auto& slot = owner_.inventoryRef().getBackpackSlot(i);
            if (slot.empty()) continue;
            uint32_t quality = 0;
            uint32_t sellPrice = slot.item.sellPrice;
            if (auto* info = owner_.getItemInfo(slot.item.itemId); info && info->valid) {
                quality = info->quality;
                if (sellPrice == 0) sellPrice = info->sellPrice;
            }
            if (quality == 0 && sellPrice > 0) {
                uint64_t itemGuid = owner_.backpackSlotGuidsRef()[i];
                if (itemGuid == 0) itemGuid = owner_.resolveOnlineItemGuid(slot.item.itemId);
                if (itemGuid != 0) {
                    sellItem(currentVendorItems_.vendorGuid, itemGuid, 1);
                    totalSellPrice += sellPrice;
                    ++itemsSold;
                }
            }
        }
        for (int b = 0; b < owner_.inventoryRef().NUM_BAG_SLOTS; ++b) {
            int bagSize = owner_.inventoryRef().getBagSize(b);
            for (int s = 0; s < bagSize; ++s) {
                const auto& slot = owner_.inventoryRef().getBagSlot(b, s);
                if (slot.empty()) continue;
                uint32_t quality = 0;
                uint32_t sellPrice = slot.item.sellPrice;
                if (auto* info = owner_.getItemInfo(slot.item.itemId); info && info->valid) {
                    quality = info->quality;
                    if (sellPrice == 0) sellPrice = info->sellPrice;
                }
                if (quality == 0 && sellPrice > 0) {
                    uint64_t itemGuid = 0;
                    uint64_t bagGuid = owner_.equipSlotGuidsRef()[19 + b];
                    if (bagGuid != 0) {
                        auto cit = owner_.containerContentsRef().find(bagGuid);
                        if (cit != owner_.containerContentsRef().end() && s < static_cast<int>(cit->second.numSlots))
                            itemGuid = cit->second.slotGuids[s];
                    }
                    if (itemGuid == 0) itemGuid = owner_.resolveOnlineItemGuid(slot.item.itemId);
                    if (itemGuid != 0) {
                        sellItem(currentVendorItems_.vendorGuid, itemGuid, 1);
                        totalSellPrice += sellPrice;
                        ++itemsSold;
                    }
                }
            }
        }
        if (itemsSold > 0) {
            uint32_t gold = totalSellPrice / 10000;
            uint32_t silver = (totalSellPrice % 10000) / 100;
            uint32_t copper = totalSellPrice % 100;
            char buf[128];
            std::snprintf(buf, sizeof(buf),
                "|cffaaaaaaAuto-sold %d grey item%s for %ug %us %uc.|r",
                itemsSold, itemsSold == 1 ? "" : "s", gold, silver, copper);
            owner_.addSystemChatMessage(buf);
        }
    }

    // Auto-repair
    if (autoRepair_ && currentVendorItems_.canRepair && currentVendorItems_.vendorGuid != 0) {
        bool anyDamaged = false;
        for (int i = 0; i < Inventory::NUM_EQUIP_SLOTS; ++i) {
            const auto& slot = owner_.inventoryRef().getEquipSlot(static_cast<EquipSlot>(i));
            if (!slot.empty() && slot.item.maxDurability > 0
                    && slot.item.curDurability < slot.item.maxDurability) {
                anyDamaged = true;
                break;
            }
        }
        if (anyDamaged) {
            repairAll(currentVendorItems_.vendorGuid, false);
            owner_.addSystemChatMessage("|cffaaaaaaAuto-repair triggered.|r");
        }
    }

    // Play vendor sound
    if (owner_.npcVendorCallbackRef() && currentVendorItems_.vendorGuid != 0) {
        auto entity = owner_.getEntityManager().getEntity(currentVendorItems_.vendorGuid);
        if (entity && entity->getType() == ObjectType::UNIT) {
            glm::vec3 pos(entity->getX(), entity->getY(), entity->getZ());
            owner_.npcVendorCallbackRef()(currentVendorItems_.vendorGuid, pos);
        }
    }

    for (const auto& item : currentVendorItems_.items) {
        owner_.queryItemInfo(item.itemId, 0);
    }
}

// ============================================================
// Trainer
// ============================================================

void InventoryHandler::handleTrainerList(network::Packet& packet) {
    const bool isClassic = isClassicLikeExpansion();
    if (!TrainerListParser::parse(packet, currentTrainerList_, isClassic)) return;
    trainerWindowOpen_ = true;
    owner_.closeGossip();
    if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("TRAINER_SHOW", {});

    LOG_INFO("Trainer list: ", currentTrainerList_.spells.size(), " spells");

    owner_.loadSpellNameCache();
    owner_.loadSkillLineDbc();
    owner_.loadSkillLineAbilityDbc();
    categorizeTrainerSpells();
}

void InventoryHandler::trainSpell(uint32_t spellId) {
    LOG_INFO("Trainer purchase requested: spellId=", spellId,
             " state=", (int)owner_.getState(),
             " socket=", (owner_.getSocket() ? "yes" : "no"));
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) {
        LOG_WARNING("trainSpell: Not in world or no socket connection");
        return;
    }

    uint32_t spellCost = 0;
    for (const auto& spell : currentTrainerList_.spells) {
        if (spell.spellId == spellId) {
            spellCost = spell.spellCost;
            break;
        }
    }
    LOG_INFO("Player money: ", owner_.playerMoneyCopperRef(), " copper, spell cost: ", spellCost, " copper");

    LOG_INFO("Sending CMSG_TRAINER_BUY_SPELL: guid=", currentTrainerList_.trainerGuid,
             " spellId=", spellId);
    auto packet = TrainerBuySpellPacket::build(
        currentTrainerList_.trainerGuid,
        spellId);
    owner_.getSocket()->send(packet);
    LOG_INFO("CMSG_TRAINER_BUY_SPELL sent");
}

void InventoryHandler::closeTrainer() {
    trainerWindowOpen_ = false;
    if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("TRAINER_CLOSED", {});
    currentTrainerList_ = TrainerListData{};
    trainerTabs_.clear();
}

void InventoryHandler::categorizeTrainerSpells() {
    trainerTabs_.clear();

    static constexpr uint32_t SKILLLINE_CATEGORY_CLASS = 7;

    std::map<uint32_t, std::vector<const TrainerSpell*>> specialtySpells;
    std::vector<const TrainerSpell*> generalSpells;

    for (const auto& spell : currentTrainerList_.spells) {
        auto slIt = owner_.spellToSkillLineRef().find(spell.spellId);
        if (slIt != owner_.spellToSkillLineRef().end()) {
            uint32_t skillLineId = slIt->second;
            auto catIt = owner_.skillLineCategoriesRef().find(skillLineId);
            if (catIt != owner_.skillLineCategoriesRef().end() && catIt->second == SKILLLINE_CATEGORY_CLASS) {
                specialtySpells[skillLineId].push_back(&spell);
                continue;
            }
        }
        generalSpells.push_back(&spell);
    }

    auto byName = [this](const TrainerSpell* a, const TrainerSpell* b) {
        return owner_.getSpellName(a->spellId) < owner_.getSpellName(b->spellId);
    };

    std::vector<std::pair<std::string, std::vector<const TrainerSpell*>>> named;
    for (auto& [skillLineId, spells] : specialtySpells) {
        auto nameIt = owner_.skillLineNamesRef().find(skillLineId);
        std::string tabName = (nameIt != owner_.skillLineNamesRef().end()) ? nameIt->second : "Specialty";
        std::sort(spells.begin(), spells.end(), byName);
        named.push_back({std::move(tabName), std::move(spells)});
    }
    std::sort(named.begin(), named.end(),
        [](const auto& a, const auto& b) { return a.first < b.first; });

    for (auto& [name, spells] : named) {
        trainerTabs_.push_back({std::move(name), std::move(spells)});
    }

    if (!generalSpells.empty()) {
        std::sort(generalSpells.begin(), generalSpells.end(), byName);
        trainerTabs_.push_back({"General", std::move(generalSpells)});
    }

    LOG_INFO("Trainer: Categorized into ", trainerTabs_.size(), " tabs");
}

// ============================================================
// Mail
// ============================================================

void InventoryHandler::openMailbox(uint64_t guid) {
    mailboxGuid_ = guid;
    mailboxOpen_ = true;
    hasNewMail_ = false;
    selectedMailIndex_ = -1;
    showMailCompose_ = false;
    clearMailAttachments();
    if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("MAIL_SHOW", {});
    refreshMailList();
}

void InventoryHandler::closeMailbox() {
    mailboxOpen_ = false;
    mailboxGuid_ = 0;
    showMailCompose_ = false;
    clearMailAttachments();
    if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("MAIL_CLOSED", {});
}

void InventoryHandler::refreshMailList() {
    if (!mailboxOpen_ || mailboxGuid_ == 0) return;
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    auto packet = GetMailListPacket::build(mailboxGuid_);
    owner_.getSocket()->send(packet);
}

void InventoryHandler::sendMail(const std::string& recipient, const std::string& subject,
                                const std::string& body, uint64_t money, uint64_t cod) {
    if (owner_.getState() != WorldState::IN_WORLD) {
        LOG_WARNING("sendMail: not in world");
        return;
    }
    if (!owner_.getSocket()) {
        LOG_WARNING("sendMail: no socket");
        return;
    }
    if (mailboxGuid_ == 0) {
        LOG_WARNING("sendMail: mailboxGuid_ is 0 (mailbox closed?)");
        return;
    }
    // Collect attached item GUIDs
    std::vector<uint64_t> itemGuids;
    for (const auto& att : mailAttachments_) {
        if (att.occupied()) {
            itemGuids.push_back(att.itemGuid);
        }
    }
    const int sendable = maxSendableMailAttachments();
    if (static_cast<int>(itemGuids.size()) > sendable) {
        // Should be unreachable now that attaching is capped, but dropping
        // attachments without saying so is how this went unnoticed.
        LOG_ERROR("sendMail: ", itemGuids.size(), " attachments but this expansion's "
                  "packet carries ", sendable, " — refusing to send and lose the rest");
        owner_.addSystemChatMessage("This realm's mail carries one item per letter.");
        return;
    }
    auto packet = owner_.getPacketParsers()->buildSendMail(mailboxGuid_, recipient, subject, body, money, cod, itemGuids);
    LOG_INFO("sendMail: to='", recipient, "' subject='", subject, "' money=", money,
             " attachments=", itemGuids.size(), " mailboxGuid=", mailboxGuid_);
    owner_.getSocket()->send(packet);
    clearMailAttachments();
}

int InventoryHandler::maxSendableMailAttachments() {
    // Vanilla's packet has one uint64 item GUID where TBC and later have a
    // count followed by an array.
    return isClassicLikeExpansion() ? 1 : MAIL_MAX_ATTACHMENTS;
}

bool InventoryHandler::attachItemFromBackpack(int backpackIndex) {
    if (backpackIndex < 0 || backpackIndex >= owner_.inventoryRef().getBackpackSize()) return false;
    const auto& slot = owner_.inventoryRef().getBackpackSlot(backpackIndex);
    if (slot.empty()) return false;
    uint64_t itemGuid = owner_.backpackSlotGuidsRef()[backpackIndex];
    if (itemGuid == 0) return false;
    for (int i = 0; i < maxSendableMailAttachments(); ++i) {
        if (!mailAttachments_[i].occupied()) {
            mailAttachments_[i].itemGuid = itemGuid;
            mailAttachments_[i].item = slot.item;
            mailAttachments_[i].srcBag = 0xFF;
            mailAttachments_[i].srcSlot = static_cast<uint8_t>(Inventory::NUM_EQUIP_SLOTS + backpackIndex);
            return true;
        }
    }
    return false;
}

bool InventoryHandler::attachItemFromBag(int bagIndex, int slotIndex) {
    if (bagIndex < 0 || bagIndex >= owner_.inventoryRef().NUM_BAG_SLOTS) return false;
    if (slotIndex < 0 || slotIndex >= owner_.inventoryRef().getBagSize(bagIndex)) return false;
    const auto& slot = owner_.inventoryRef().getBagSlot(bagIndex, slotIndex);
    if (slot.empty()) return false;
    uint64_t bagGuid = owner_.equipSlotGuidsRef()[Inventory::FIRST_BAG_EQUIP_SLOT + bagIndex];
    if (bagGuid == 0) return false;
    auto it = owner_.containerContentsRef().find(bagGuid);
    if (it == owner_.containerContentsRef().end()) return false;
    if (slotIndex >= static_cast<int>(it->second.numSlots)) return false;
    uint64_t itemGuid = it->second.slotGuids[slotIndex];
    if (itemGuid == 0) return false;
    for (int i = 0; i < maxSendableMailAttachments(); ++i) {
        if (!mailAttachments_[i].occupied()) {
            mailAttachments_[i].itemGuid = itemGuid;
            mailAttachments_[i].item = slot.item;
            mailAttachments_[i].srcBag = static_cast<uint8_t>(Inventory::FIRST_BAG_EQUIP_SLOT + bagIndex);
            mailAttachments_[i].srcSlot = static_cast<uint8_t>(slotIndex);
            return true;
        }
    }
    return false;
}

bool InventoryHandler::detachMailAttachment(int attachIndex) {
    if (attachIndex < 0 || attachIndex >= MAIL_MAX_ATTACHMENTS) return false;
    mailAttachments_[attachIndex] = MailAttachSlot{};
    return true;
}

void InventoryHandler::clearMailAttachments() {
    for (auto& a : mailAttachments_) a = MailAttachSlot{};
}

int InventoryHandler::getMailAttachmentCount() const {
    int count = 0;
    for (const auto& a : mailAttachments_)
        if (a.occupied()) ++count;
    return count;
}

void InventoryHandler::mailTakeMoney(uint32_t mailId) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket() || mailboxGuid_ == 0) return;
    auto packet = MailTakeMoneyPacket::build(mailboxGuid_, mailId);
    owner_.getSocket()->send(packet);
}

void InventoryHandler::mailTakeItem(uint32_t mailId, uint32_t itemGuidLow) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket() || mailboxGuid_ == 0) return;
    auto packet = MailTakeItemPacket::build(mailboxGuid_, mailId, itemGuidLow);
    owner_.getSocket()->send(packet);
}

void InventoryHandler::mailDelete(uint32_t mailId) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket() || mailboxGuid_ == 0) return;
    auto packet = MailDeletePacket::build(mailboxGuid_, mailId, 0);
    owner_.getSocket()->send(packet);
}

void InventoryHandler::mailMarkAsRead(uint32_t mailId) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket() || mailboxGuid_ == 0) return;
    auto packet = MailMarkAsReadPacket::build(mailboxGuid_, mailId);
    owner_.getSocket()->send(packet);
}

void InventoryHandler::handleShowMailbox(network::Packet& packet) {
    if (!packet.hasRemaining(8)) return;
    mailboxGuid_ = packet.readUInt64();
    mailboxOpen_ = true;
    selectedMailIndex_ = -1;
    if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("MAIL_SHOW", {});
    refreshMailList();
}

void InventoryHandler::handleMailListResult(network::Packet& packet) {
    if (!owner_.getPacketParsers()->parseMailList(packet, mailInbox_)) return;
    if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("MAIL_INBOX_UPDATE", {});
    for (const auto& mail : mailInbox_) {
        // Player mail carries a GUID, not a name. Ask for it once here; the UI
        // reads the name through GameHandler::getMailSenderName, so it appears
        // as soon as the query comes back.
        if (mail.messageType == 0 && mail.senderGuid != 0 &&
            owner_.lookupName(mail.senderGuid).empty()) {
            owner_.queryPlayerName(mail.senderGuid);
        }
        if (mail.messageType == 2) {
            AuctionMailSubject auction;
            if (parseAuctionMailSubject(mail.subject, auction)) {
                owner_.ensureItemInfo(auction.itemEntry);
            }
        }
        for (const auto& att : mail.attachments) {
            if (att.itemId != 0) owner_.ensureItemInfo(att.itemId);
        }
    }
}

void InventoryHandler::handleSendMailResult(network::Packet& packet) {
    if (!packet.hasRemaining(12)) return;
    uint32_t mailId = packet.readUInt32();
    uint32_t action = packet.readUInt32();
    uint32_t error  = packet.readUInt32();
    // WotLK MailResponseType values from SharedDefines.h.
    constexpr uint32_t MAIL_SEND = 0;
    constexpr uint32_t MAIL_MONEY_TAKEN = 1;
    constexpr uint32_t MAIL_ITEM_TAKEN = 2;
    constexpr uint32_t MAIL_RETURNED_TO_SENDER = 3;
    constexpr uint32_t MAIL_DELETED = 4;
    constexpr uint32_t MAIL_MADE_PERMANENT = 5;

    auto mailIt = std::find_if(mailInbox_.begin(), mailInbox_.end(),
        [mailId](const MailMessage& mail) { return mail.messageId == mailId; });

    if (action == MAIL_SEND) {
        if (error == 0) {
            owner_.addSystemChatMessage("Mail sent.");
            clearMailAttachments();
            showMailCompose_ = false;
        } else {
            owner_.addSystemChatMessage("Failed to send mail (error " + std::to_string(error) + ").");
        }
    } else if (action == MAIL_ITEM_TAKEN) {
        if (error == 0) {
            owner_.addSystemChatMessage("Item taken from mail.");
            if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("BAG_UPDATE", {});
        } else {
            owner_.addSystemChatMessage("Failed to take item (error " + std::to_string(error) + ").");
        }
    } else if (action == MAIL_MONEY_TAKEN) {
        if (error == 0) {
            if (mailIt != mailInbox_.end()) mailIt->money = 0;
            owner_.addSystemChatMessage("Money taken from mail.");
            if (owner_.addonEventCallbackRef()) {
                owner_.addonEventCallbackRef()("PLAYER_MONEY", {});
                owner_.addonEventCallbackRef()("MAIL_INBOX_UPDATE", {});
            }
        } else {
            owner_.addSystemChatMessage("Failed to take money (error " + std::to_string(error) + ").");
        }
    } else if (action == MAIL_DELETED || action == MAIL_RETURNED_TO_SENDER) {
        if (error == 0) {
            owner_.addSystemChatMessage(action == MAIL_DELETED ? "Mail deleted." : "Mail returned.");
            if (mailIt != mailInbox_.end()) {
                const int erasedIndex = static_cast<int>(std::distance(mailInbox_.begin(), mailIt));
                mailInbox_.erase(mailIt);
                if (selectedMailIndex_ == erasedIndex) selectedMailIndex_ = -1;
                else if (selectedMailIndex_ > erasedIndex) --selectedMailIndex_;
            }
            if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("MAIL_INBOX_UPDATE", {});
        }
    } else if (action == MAIL_MADE_PERMANENT && error == 0) {
        owner_.addSystemChatMessage("Mail text copied.");
    }
    refreshMailList();
}

void InventoryHandler::handleReceivedMail(network::Packet& packet) {
    (void)packet;
    setHasNewMail(true);
}

void InventoryHandler::handleQueryNextMailTime(network::Packet& packet) {
    if (!packet.hasRemaining(8)) return;
    // readFloat() uses memcpy internally, avoiding the strict aliasing violation
    // that the previous reinterpret_cast<float*> on raw packet bytes had.
    float nextTime = packet.readFloat();
    uint32_t count = packet.readUInt32();
    setHasNewMail(nextTime >= 0.0f && count > 0);
    packet.skipAll();
}

void InventoryHandler::setHasNewMail(bool value) {
    // Announce a chat line + sound only on the rising edge (no unread -> unread), so the
    // periodic next-mail-time poll doesn't repeat the notification while mail sits unread.
    if (value && !hasNewMail_) {
        owner_.addSystemChatMessage("You have new mail.");
        if (auto* ac = owner_.services().audioCoordinator)
            if (auto* sfx = ac->getUiSoundManager()) sfx->playMailReceived();
    }
    hasNewMail_ = value;
    if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("UPDATE_PENDING_MAIL", {});
}

// ============================================================
// Bank
// ============================================================

void InventoryHandler::openBank(uint64_t guid) {
    bankerGuid_ = guid;
    bankOpen_ = true;
    if (isClassicLikeExpansion()) {
        effectiveBankSlots_ = 24;
        effectiveBankBagSlots_ = 6;
    } else {
        effectiveBankSlots_ = 28;
        effectiveBankBagSlots_ = 7;
    }
    if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("BANKFRAME_OPENED", {});
}

void InventoryHandler::closeBank() {
    bankOpen_ = false;
    bankerGuid_ = 0;
    if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("BANKFRAME_CLOSED", {});
}

void InventoryHandler::buyBankSlot() {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket() || bankerGuid_ == 0) return;
    auto packet = BuyBankSlotPacket::build(bankerGuid_);
    owner_.getSocket()->send(packet);
}

uint32_t InventoryHandler::getBankBagSlotPrice(int slotIndex) {
    // BankBagSlotPrices.dbc — copper cost for each successive bank bag slot.
    // These values are stable across Classic 1.12, TBC 2.4.3, and WotLK 3.3.5a.
    static constexpr uint32_t kPrices[Inventory::BANK_BAG_SLOTS] = {
        10000,    //   1g  — 1st slot
        100000,   //  10g  — 2nd slot
        250000,   //  25g  — 3rd slot
        600000,   //  60g  — 4th slot
        1000000,  // 100g  — 5th slot
        2500000,  // 250g  — 6th slot
        5000000,  // 500g  — 7th slot
    };
    if (slotIndex < 0 || slotIndex >= Inventory::BANK_BAG_SLOTS) return 0;
    return kPrices[slotIndex];
}

void InventoryHandler::depositItem(uint8_t srcBag, uint8_t srcSlot) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    // CMSG_AUTOBANK_ITEM lets the server place the item into the first free slot
    // across the whole bank — the main slots AND the purchased bank bags. The
    // old code scanned only the main bank slots and reported "Bank is full" the
    // moment those filled, ignoring free space in the bank bags. The server
    // replies with a bank-full error if there is genuinely no room.
    auto packet = AutoBankItemPacket::build(srcBag, srcSlot);
    owner_.getSocket()->send(packet);
}

void InventoryHandler::withdrawItem(uint8_t srcBag, uint8_t srcSlot) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    // CMSG_AUTOSTORE_BANK_ITEM lets the server place the item into the first
    // free slot across ALL bags (retail right-click-to-withdraw), not just the
    // backpack. The server replies with an inventory-full error if there's no
    // room, so no client-side capacity check is needed here.
    auto packet = AutoStoreBankItemPacket::build(srcBag, srcSlot);
    owner_.getSocket()->send(packet);
}

void InventoryHandler::handleShowBank(network::Packet& packet) {
    if (!packet.hasRemaining(8)) return;
    uint64_t guid = packet.readUInt64();
    openBank(guid);
}

void InventoryHandler::handleBuyBankSlotResult(network::Packet& packet) {
    if (!packet.hasRemaining(4)) return;
    uint32_t result = packet.readUInt32();
    if (result == 0) {
        owner_.addSystemChatMessage("Bank slot purchased.");
        if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("PLAYERBANKBAGSLOTS_CHANGED", {});
    } else {
        owner_.addSystemChatMessage("Failed to purchase bank slot.");
    }
}

// ============================================================
// Guild Bank
// ============================================================

void InventoryHandler::openGuildBank(uint64_t guid) {
    guildBankerGuid_ = guid;
    guildBankOpen_ = true;
    guildBankActiveTab_ = 0;
    if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("GUILDBANKFRAME_OPENED", {});
    // CMSG_GUILD_BANKER_ACTIVATE registers this session with the guild banker so
    // the server sends the tab list + contents (SMSG_GUILD_BANK_LIST). Without it
    // the follow-up query-tab is rejected and no bank list ever arrives, so the
    // window opened empty (or not at all).
    if (owner_.getState() == WorldState::IN_WORLD && owner_.getSocket() && guid != 0) {
        auto activate = GuildBankerActivatePacket::build(guid);
        owner_.getSocket()->send(activate);
    }
    queryGuildBankTab(0);
}

void InventoryHandler::closeGuildBank() {
    guildBankOpen_ = false;
    guildBankerGuid_ = 0;
    if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("GUILDBANKFRAME_CLOSED", {});
}

void InventoryHandler::queryGuildBankTab(uint8_t tabId) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket() || guildBankerGuid_ == 0) return;
    auto packet = GuildBankQueryTabPacket::build(guildBankerGuid_, tabId, false);
    owner_.getSocket()->send(packet);
}

void InventoryHandler::buyGuildBankTab() {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket() || guildBankerGuid_ == 0) return;
    uint8_t nextTab = static_cast<uint8_t>(guildBankData_.tabs.size());
    auto packet = GuildBankBuyTabPacket::build(guildBankerGuid_, nextTab);
    owner_.getSocket()->send(packet);
}

void InventoryHandler::depositGuildBankMoney(uint32_t amount) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket() || guildBankerGuid_ == 0) return;
    auto packet = GuildBankDepositMoneyPacket::build(guildBankerGuid_, amount);
    owner_.getSocket()->send(packet);
}

void InventoryHandler::withdrawGuildBankMoney(uint32_t amount) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket() || guildBankerGuid_ == 0) return;
    auto packet = GuildBankWithdrawMoneyPacket::build(guildBankerGuid_, amount);
    owner_.getSocket()->send(packet);
}

void InventoryHandler::guildBankWithdrawItem(uint8_t tabId, uint8_t bankSlot, uint8_t destBag, uint8_t destSlot) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket() || guildBankerGuid_ == 0) return;
    auto packet = GuildBankSwapItemsPacket::buildBankToInventory(guildBankerGuid_, tabId, bankSlot, destBag, destSlot);
    owner_.getSocket()->send(packet);
}

void InventoryHandler::guildBankDepositItem(uint8_t tabId, uint8_t bankSlot, uint8_t srcBag, uint8_t srcSlot) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket() || guildBankerGuid_ == 0) return;
    auto packet = GuildBankSwapItemsPacket::buildInventoryToBank(guildBankerGuid_, tabId, bankSlot, srcBag, srcSlot);
    owner_.getSocket()->send(packet);
}

void InventoryHandler::guildBankDepositFromInventory(uint8_t srcBag, uint8_t srcSlot) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket() ||
        guildBankerGuid_ == 0 || !guildBankOpen_) return;
    // CMSG_GUILD_BANK_SWAP_ITEMS has no server-side auto-store for the deposit
    // direction (that path forces bank→character), so the client picks the
    // target slot — the first empty one in the tab the player is viewing, which
    // is what the retail client does on a right-click deposit.
    constexpr int kTabSlots = 98; // GUILD_BANK_MAX_SLOTS
    std::array<bool, kTabSlots> occupied{};
    for (const auto& slot : guildBankData_.tabItems) {
        if (slot.itemEntry != 0 && slot.slotId < kTabSlots)
            occupied[slot.slotId] = true;
    }
    int freeSlot = -1;
    for (int s = 0; s < kTabSlots; ++s) {
        if (!occupied[s]) { freeSlot = s; break; }
    }
    if (freeSlot < 0) {
        owner_.addSystemChatMessage("This guild bank tab is full.");
        return;
    }
    guildBankDepositItem(guildBankActiveTab_, static_cast<uint8_t>(freeSlot), srcBag, srcSlot);
}

void InventoryHandler::handleGuildBankList(network::Packet& packet) {
    if (!GuildBankListParser::parse(packet, guildBankData_)) return;
    // Receiving the bank list means the banker accepted us — make sure the
    // window is shown even if the open path didn't (e.g. a server-initiated
    // refresh, or the banker guid arrived only with the list).
    if (guildBankerGuid_ != 0 && !guildBankOpen_) {
        guildBankOpen_ = true;
        if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("GUILDBANKFRAME_OPENED", {});
    }
    // Each list is tagged with the tab it describes. Track it so the UI
    // highlights the right tab and item withdraw/deposit target the tab the
    // player is actually viewing (clicking a tab only sends a query — it never
    // updated the active tab, so operations defaulted to tab 0).
    guildBankActiveTab_ = guildBankData_.tabId;
    if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("GUILDBANKBAGSLOTS_CHANGED", {});
    for (const auto& tab : guildBankData_.tabs) {
        for (const auto& item : tab.items) {
            if (item.itemEntry != 0) owner_.ensureItemInfo(item.itemEntry);
        }
    }
}

// ============================================================
// Auction House
// ============================================================

void InventoryHandler::openAuctionHouse(uint64_t guid) {
    auctioneerGuid_ = guid;
    auctionOpen_ = true;
    auctionActiveTab_ = 0;
    if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("AUCTION_HOUSE_SHOW", {});
}

void InventoryHandler::closeAuctionHouse() {
    auctionOpen_ = false;
    auctioneerGuid_ = 0;
    if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("AUCTION_HOUSE_CLOSED", {});
}

void InventoryHandler::auctionSearch(const std::string& name, uint8_t levelMin, uint8_t levelMax,
                                      uint32_t quality, uint32_t itemClass, uint32_t itemSubClass,
                                      uint32_t invTypeMask, uint8_t usableOnly, uint32_t offset) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket() || auctioneerGuid_ == 0) return;
    lastAuctionSearch_ = {name, levelMin, levelMax, quality, itemClass, itemSubClass, invTypeMask, usableOnly, offset};
    hasAuctionSearch_ = true;
    pendingAuctionTarget_ = AuctionResultTarget::BROWSE;
    auto packet = AuctionListItemsPacket::build(auctioneerGuid_, offset, name,
                                                  levelMin, levelMax, invTypeMask,
                                                  itemClass, itemSubClass, quality, usableOnly, 0);
    owner_.getSocket()->send(packet);
    auctionSearchDelayTimer_ = 5.0f;
}

void InventoryHandler::auctionSellItem(int backpackIndex, uint32_t bid,
                                        uint32_t buyout, uint32_t duration) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket() || auctioneerGuid_ == 0) return;
    if (backpackIndex < 0 || backpackIndex >= owner_.inventoryRef().getBackpackSize()) return;
    const auto& slot = owner_.inventoryRef().getBackpackSlot(backpackIndex);
    if (slot.empty()) return;

    uint64_t itemGuid = owner_.backpackSlotGuidsRef()[backpackIndex];
    if (itemGuid == 0) {
        itemGuid = owner_.resolveOnlineItemGuid(slot.item.itemId);
    }
    if (itemGuid == 0) {
        LOG_ERROR("auctionSellItem: could not resolve GUID for backpack slot ", backpackIndex);
        return;
    }

    uint32_t stackCount = slot.item.stackCount;
    auto packet = AuctionSellItemPacket::build(auctioneerGuid_, itemGuid, stackCount, bid, buyout, duration,
                                                isPreWotlk());
    owner_.getSocket()->send(packet);
}

void InventoryHandler::auctionSellItemByGuid(uint64_t itemGuid, uint32_t stackCount, uint32_t bid,
                                             uint32_t buyout, uint32_t duration) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket() || auctioneerGuid_ == 0) return;
    if (itemGuid == 0) {
        LOG_ERROR("auctionSellItemByGuid: refusing to post with a null item GUID");
        return;
    }
    if (stackCount == 0) stackCount = 1;
    auto packet = AuctionSellItemPacket::build(auctioneerGuid_, itemGuid, stackCount, bid, buyout, duration,
                                                isPreWotlk());
    owner_.getSocket()->send(packet);
}

void InventoryHandler::auctionPlaceBid(uint32_t auctionId, uint32_t amount) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket() || auctioneerGuid_ == 0) return;
    auto packet = AuctionPlaceBidPacket::build(auctioneerGuid_, auctionId, amount);
    owner_.getSocket()->send(packet);
}

void InventoryHandler::auctionBuyout(uint32_t auctionId, uint32_t buyoutPrice) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket() || auctioneerGuid_ == 0) return;
    auto packet = AuctionPlaceBidPacket::build(auctioneerGuid_, auctionId, buyoutPrice);
    owner_.getSocket()->send(packet);
}

void InventoryHandler::auctionCancelItem(uint32_t auctionId) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket() || auctioneerGuid_ == 0) return;
    auto packet = AuctionRemoveItemPacket::build(auctioneerGuid_, auctionId);
    owner_.getSocket()->send(packet);
}

void InventoryHandler::auctionListOwnerItems(uint32_t offset) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket() || auctioneerGuid_ == 0) return;
    pendingAuctionTarget_ = AuctionResultTarget::OWNER;
    auto packet = AuctionListOwnerItemsPacket::build(auctioneerGuid_, offset);
    owner_.getSocket()->send(packet);
}

void InventoryHandler::auctionListBidderItems(uint32_t offset) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket() || auctioneerGuid_ == 0) return;
    pendingAuctionTarget_ = AuctionResultTarget::BIDDER;
    auto packet = AuctionListBidderItemsPacket::build(auctioneerGuid_, offset);
    owner_.getSocket()->send(packet);
}

void InventoryHandler::handleAuctionHello(network::Packet& packet) {
    if (!packet.hasRemaining(12)) return;
    uint64_t guid = packet.readUInt64();
    uint32_t houseId = packet.readUInt32();
    auctioneerGuid_ = guid;
    auctionHouseId_ = houseId;
    auctionOpen_ = true;
    auctionActiveTab_ = 0;
    owner_.closeGossip();
    if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("AUCTION_HOUSE_SHOW", {});
}

void InventoryHandler::handleAuctionListResult(network::Packet& packet) {
    AuctionListResult result;
    // Vanilla sends a single enchantId; TBC inspects 6 enchant slots per item,
    // WotLK 7 (PRISMATIC_ENCHANTMENT_SLOT joined the inspected range in 3.x).
    const int enchantSlots = isClassicLikeExpansion() ? 1 : (isPreWotlk() ? 6 : 7);
    if (!AuctionListResultParser::parse(packet, result, enchantSlots)) return;

    if (pendingAuctionTarget_ == AuctionResultTarget::OWNER) {
        auctionOwnerResults_ = std::move(result);
        if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("AUCTION_OWNED_LIST_UPDATE", {});
    } else if (pendingAuctionTarget_ == AuctionResultTarget::BIDDER) {
        auctionBidderResults_ = std::move(result);
        if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("AUCTION_BIDDER_LIST_UPDATE", {});
    } else {
        auctionBrowseResults_ = std::move(result);
        if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("AUCTION_ITEM_LIST_UPDATE", {});
    }

    // Ensure item info for all entries
    auto ensureEntries = [this](const AuctionListResult& r) {
        for (const auto& e : r.auctions) {
            owner_.ensureItemInfo(e.itemEntry);
            if (e.ownerGuid != 0) owner_.queryPlayerName(e.ownerGuid);
        }
    };
    if (pendingAuctionTarget_ == AuctionResultTarget::OWNER) ensureEntries(auctionOwnerResults_);
    else if (pendingAuctionTarget_ == AuctionResultTarget::BIDDER) ensureEntries(auctionBidderResults_);
    else ensureEntries(auctionBrowseResults_);
}

void InventoryHandler::handleAuctionOwnerListResult(network::Packet& packet) {
    pendingAuctionTarget_ = AuctionResultTarget::OWNER;
    handleAuctionListResult(packet);
}

void InventoryHandler::handleAuctionBidderListResult(network::Packet& packet) {
    pendingAuctionTarget_ = AuctionResultTarget::BIDDER;
    handleAuctionListResult(packet);
}

void InventoryHandler::handleAuctionCommandResult(network::Packet& packet) {
    if (!packet.hasRemaining(12)) return;
    uint32_t auctionId = packet.readUInt32();
    uint32_t action = packet.readUInt32();
    uint32_t error  = packet.readUInt32();
    (void)auctionId;

    const char* actionNames[] = {"sell", "cancel", "bid/buyout"};
    const char* actionStr = (action < 3) ? actionNames[action] : "unknown";

    if (error == 0) {
        std::string msg = std::string("Auction ") + actionStr + " successful.";
        owner_.addSystemChatMessage(msg);
        if (owner_.addonEventCallbackRef()) {
            owner_.addonEventCallbackRef()("PLAYER_MONEY", {});
            owner_.addonEventCallbackRef()("BAG_UPDATE", {});
        }
        // Re-query after successful buy/bid so the list reflects the change.
        // Previously gated on name.length()>0 which skipped browse-all (empty name).
        if (action == 2 && hasAuctionSearch_) {
            auctionSearch(lastAuctionSearch_.name, lastAuctionSearch_.levelMin, lastAuctionSearch_.levelMax,
                         lastAuctionSearch_.quality, lastAuctionSearch_.itemClass, lastAuctionSearch_.itemSubClass,
                         lastAuctionSearch_.invTypeMask, lastAuctionSearch_.usableOnly, lastAuctionSearch_.offset);
        }
    } else {
        const char* errMsg = "Unknown error.";
        switch (error) {
            case 1: errMsg = "Not enough money."; break;
            case 2: errMsg = "Item not found."; break;
            case 5: errMsg = "Bid too low."; break;
            case 6: errMsg = "Bid increment too low."; break;
            case 7: errMsg = "You cannot bid on your own auction."; break;
            case 8: errMsg = "Database error."; break;
            default: break;
        }
        owner_.addUIError(std::string("Auction ") + actionStr + " failed: " + errMsg);
        owner_.addSystemChatMessage(std::string("Auction ") + actionStr + " failed: " + errMsg);
    }
}

// ============================================================
// Item Text
// ============================================================

void InventoryHandler::queryItemText(uint64_t itemGuid) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    network::Packet pkt(wireOpcode(Opcode::CMSG_ITEM_TEXT_QUERY));
    pkt.writeUInt64(itemGuid);
    owner_.getSocket()->send(pkt);
}

void InventoryHandler::handleItemTextQueryResponse(network::Packet& packet) {
    if (!packet.hasRemaining(1)) return;
    std::string text = packet.readString();
    if (!text.empty()) {
        itemText_ = std::move(text);
        itemTextOpen_ = true;
    }
}

// ============================================================
// Trade
// ============================================================

void InventoryHandler::acceptTradeRequest() {
    if (tradeStatus_ != TradeStatus::PendingIncoming) return;
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    auto packet = BeginTradePacket::build();
    owner_.getSocket()->send(packet);
}

void InventoryHandler::declineTradeRequest() {
    if (tradeStatus_ != TradeStatus::PendingIncoming) return;
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    auto packet = CancelTradePacket::build();
    owner_.getSocket()->send(packet);
    resetTradeState();
}

void InventoryHandler::acceptTrade() {
    if (tradeStatus_ != TradeStatus::Open) return;
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    auto packet = AcceptTradePacket::build();
    owner_.getSocket()->send(packet);
}

void InventoryHandler::cancelTrade() {
    if (tradeStatus_ == TradeStatus::None) return;
    if (owner_.getState() == WorldState::IN_WORLD && owner_.getSocket()) {
        auto packet = CancelTradePacket::build();
        owner_.getSocket()->send(packet);
    }
    resetTradeState();
}

void InventoryHandler::setTradeItem(uint8_t tradeSlot, uint8_t srcBag, uint8_t srcSlot) {
    if (tradeStatus_ != TradeStatus::Open) return;
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    auto packet = SetTradeItemPacket::build(tradeSlot, srcBag, srcSlot);
    owner_.getSocket()->send(packet);
}

void InventoryHandler::clearTradeItem(uint8_t tradeSlot) {
    if (tradeStatus_ != TradeStatus::Open) return;
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    auto packet = ClearTradeItemPacket::build(tradeSlot);
    owner_.getSocket()->send(packet);
}

void InventoryHandler::setTradeGold(uint64_t amount) {
    if (tradeStatus_ != TradeStatus::Open) return;
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    auto packet = SetTradeGoldPacket::build(static_cast<uint32_t>(amount));
    owner_.getSocket()->send(packet);
}

void InventoryHandler::resetTradeState() {
    tradeStatus_ = TradeStatus::None;
    tradePeerGuid_ = 0;
    tradePeerName_.clear();
    myTradeSlots_ = {};
    peerTradeSlots_ = {};
    myTradeGold_ = 0;
    peerTradeGold_ = 0;
}

void InventoryHandler::handleTradeStatus(network::Packet& packet) {
    if (!packet.hasRemaining(4)) return;
    uint32_t status = packet.readUInt32();
    LOG_WARNING("SMSG_TRADE_STATUS: status=", status, " size=", packet.getSize());
    switch (status) {
        case 0: // TRADE_STATUS_PLAYER_BUSY
            resetTradeState();
            owner_.addSystemChatMessage("Trade failed: player is busy.");
            break;
        case 1: { // TRADE_STATUS_PROPOSED
            if (packet.hasRemaining(8))
                tradePeerGuid_ = packet.readUInt64();
            tradeStatus_ = TradeStatus::PendingIncoming;
            // Resolve name
            auto nit = owner_.getPlayerNameCache().find(tradePeerGuid_);
            if (nit != owner_.getPlayerNameCache().end()) tradePeerName_ = nit->second;
            else tradePeerName_ = "Unknown";
            owner_.addSystemChatMessage(tradePeerName_ + " wants to trade with you.");
            if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("TRADE_REQUEST", {tradePeerName_});
            break;
        }
        case 2: // TRADE_STATUS_INITIATED
            tradeStatus_ = TradeStatus::Open;
            owner_.addSystemChatMessage("Trade opened.");
            if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("TRADE_SHOW", {});
            break;
        case 3: // TRADE_STATUS_CANCELLED
            resetTradeState();
            owner_.addSystemChatMessage("Trade cancelled.");
            if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("TRADE_CLOSED", {});
            break;
        case 4: // TRADE_STATUS_ACCEPTED
            tradeStatus_ = TradeStatus::Accepted;
            owner_.addSystemChatMessage("Trade partner accepted.");
            if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("TRADE_ACCEPT_UPDATE", {});
            break;
        case 5: // TRADE_STATUS_ALREADY_TRADING
            owner_.addSystemChatMessage("You are already trading.");
            break;
        case 7: // TRADE_STATUS_COMPLETE
            // Don't reset immediately — TRADE_STATUS_EXTENDED may arrive in the same
            // packet batch and needs the trade state to store final item/gold data.
            tradeStatus_ = TradeStatus::None;
            owner_.addSystemChatMessage("Trade complete.");
            if (owner_.addonEventCallbackRef()) {
                owner_.addonEventCallbackRef()("TRADE_CLOSED", {});
                owner_.addonEventCallbackRef()("BAG_UPDATE", {});
                owner_.addonEventCallbackRef()("PLAYER_MONEY", {});
            }
            break;
        case 9: // TRADE_STATUS_TARGET_TO_FAR
            resetTradeState();
            owner_.addSystemChatMessage("Trade failed: target is too far away.");
            break;
        case 13: // TRADE_STATUS_FAILED
            resetTradeState();
            owner_.addSystemChatMessage("Trade failed.");
            if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("TRADE_CLOSED", {});
            break;
        case 8: // TRADE_STATUS_UNACCEPT
            tradeStatus_ = TradeStatus::Open;
            if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("TRADE_ACCEPT_UPDATE", {});
            break;
        case 17: // TRADE_STATUS_PETITION
            owner_.addSystemChatMessage("You cannot trade while petition is active.");
            break;
        case 18: // TRADE_STATUS_PLAYER_IGNORED
            owner_.addSystemChatMessage("That player is ignoring you.");
            break;
        default:
            LOG_DEBUG("Unhandled SMSG_TRADE_STATUS: ", status);
            break;
    }
}

void InventoryHandler::handleTradeStatusExtended(network::Packet& packet) {
    LOG_WARNING("SMSG_TRADE_STATUS_EXTENDED: size=", packet.getSize(),
                " readPos=", packet.getReadPos());
    // WotLK format: whichPlayer(4) + tradeCount(4) + N items × (slot(1)+64bytes) + gold(4)
    // Total for empty trade: 8 + 8×65 + 4 = 532 (matches observed packet size)
    if (!packet.hasRemaining(8)) return;
    uint32_t whichPlayer = packet.readUInt32();   // 0=self, 1=peer (uint32 not uint8!)
    uint32_t tradeCount = packet.readUInt32();
    auto& slots = (whichPlayer == 0) ? myTradeSlots_ : peerTradeSlots_;
    if (tradeCount > TRADE_SLOT_COUNT) tradeCount = TRADE_SLOT_COUNT;  // 6 traded + 1 non-traded slot
    LOG_WARNING("  whichPlayer=", whichPlayer, " tradeCount=", tradeCount);

    for (uint32_t i = 0; i < tradeCount; ++i) {
        if (!packet.hasRemaining(1)) break;
        uint8_t slotNum = packet.readUInt8();
        // Per-slot: 4(item)+4(display)+4(stack)+4(wrapped)+8(creator)
        //           +4(enchant)+3×4(gems)+4(maxDur)+4(dur)+4(spellCharges)
        //           +4(suffixFactor)+4(randomPropId)+4(lockId) = 64 bytes
        if (!packet.hasRemaining(64)) { packet.skipAll(); return; }
        uint32_t itemId    = packet.readUInt32();
        uint32_t displayId = packet.readUInt32();
        uint32_t stackCnt  = packet.readUInt32();
        /*uint32_t wrapped =*/ packet.readUInt32();
        /*uint64_t giftCreator =*/ packet.readUInt64();
        /*uint32_t enchant =*/ packet.readUInt32();
        for (int g = 0; g < 3; ++g) packet.readUInt32(); // socket enchant IDs
        /*uint32_t maxDur =*/ packet.readUInt32();
        /*uint32_t curDur =*/ packet.readUInt32();
        /*uint32_t spellCharges =*/ packet.readUInt32();
        /*uint32_t suffixFactor =*/ packet.readUInt32();
        /*uint32_t randomPropId =*/ packet.readUInt32();
        /*uint32_t lockId =*/ packet.readUInt32();

        if (slotNum < TRADE_SLOT_COUNT) {
            slots[slotNum].itemId = itemId;
            slots[slotNum].displayId = displayId;
            slots[slotNum].stackCount = stackCnt;
        }
        if (itemId != 0) owner_.ensureItemInfo(itemId);
    }

    // Gold
    if (packet.hasRemaining(4)) {
        uint32_t gold = packet.readUInt32();
        if (whichPlayer == 0) myTradeGold_ = gold;
        else peerTradeGold_ = gold;
    }

    if (owner_.addonEventCallbackRef()) owner_.addonEventCallbackRef()("TRADE_UPDATE", {});
}

// ============================================================
// Equipment Sets
// ============================================================

bool InventoryHandler::supportsEquipmentSets() const {
    return isActiveExpansion("wotlk");
}

void InventoryHandler::useEquipmentSet(uint32_t setId) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    uint16_t wire = wireOpcode(Opcode::CMSG_EQUIPMENT_SET_USE);
    if (wire == 0xFFFF) { owner_.addUIError("Equipment sets not supported."); return; }
    const EquipmentSet* es = nullptr;
    for (const auto& s : equipmentSets_) {
        if (s.setId == setId) { es = &s; break; }
    }
    if (!es) {
        owner_.addSystemChatMessage("Equipment set not found.");
        return;
    }
    network::Packet pkt(wire);
    for (int slot = 0; slot < 19; ++slot) {
        uint64_t itemGuid = es->itemGuids[slot];
        pkt.writePackedGuid(itemGuid);
        uint8_t srcBag = 0xFF;
        uint8_t srcSlot = 0;
        if (itemGuid != 0) {
            bool found = false;
            for (int eq = 0; eq < 19 && !found; ++eq) {
                if (owner_.getEquipSlotGuid(eq) == itemGuid) {
                    srcBag = 0xFF;
                    srcSlot = static_cast<uint8_t>(eq);
                    found = true;
                }
            }
            for (int bp = 0; bp < 16 && !found; ++bp) {
                if (owner_.getBackpackItemGuid(bp) == itemGuid) {
                    srcBag = 0xFF;
                    srcSlot = static_cast<uint8_t>(Inventory::NUM_EQUIP_SLOTS + bp);
                    found = true;
                }
            }
            for (int bag = 0; bag < 4 && !found; ++bag) {
                int bagSize = owner_.inventoryRef().getBagSize(bag);
                for (int s = 0; s < bagSize && !found; ++s) {
                    if (owner_.getBagItemGuid(bag, s) == itemGuid) {
                        srcBag = static_cast<uint8_t>(Inventory::FIRST_BAG_EQUIP_SLOT + bag);
                        srcSlot = static_cast<uint8_t>(s);
                        found = true;
                    }
                }
            }
        }
        pkt.writeUInt8(srcBag);
        pkt.writeUInt8(srcSlot);
    }
    owner_.getSocket()->send(pkt);
}

void InventoryHandler::saveEquipmentSet(const std::string& name, const std::string& iconName,
                                         uint64_t existingGuid, uint32_t setIndex) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    uint16_t wire = wireOpcode(Opcode::CMSG_EQUIPMENT_SET_SAVE);
    if (wire == 0xFFFF) { owner_.addUIError("Equipment sets not supported."); return; }
    pendingSaveSetName_ = name;
    pendingSaveSetIcon_ = iconName;
    if (setIndex == 0xFFFFFFFF) {
        setIndex = 0;
        for (const auto& es : equipmentSets_) {
            if (es.setId >= setIndex) setIndex = es.setId + 1;
        }
    }
    network::Packet pkt(wire);
    // Packed, like everything guid-shaped in this family.
    // HandleEquipmentSetSave opens with readPackGUID, so a fixed eight bytes
    // here left seven behind: the server read the index out of the padding,
    // the name out of what was left of it, and the nineteen item guids from
    // wherever it had got to. Saving a set has never worked.
    pkt.writePackedGuid(existingGuid);
    pkt.writeUInt32(setIndex);
    pkt.writeString(name);
    pkt.writeString(iconName);
    for (int i = 0; i < 19; ++i) {
        pkt.writePackedGuid(owner_.getEquipSlotGuid(i));
    }
    owner_.getSocket()->send(pkt);
}

void InventoryHandler::deleteEquipmentSet(uint64_t setGuid) {
    if (owner_.getState() != WorldState::IN_WORLD || !owner_.getSocket()) return;
    uint16_t wire = wireOpcode(Opcode::CMSG_DELETEEQUIPMENT_SET);
    if (wire == 0xFFFF) { owner_.addUIError("Equipment sets not supported."); return; }
    network::Packet pkt(wire);
    // HandleEquipmentSetDelete reads this packed too.
    pkt.writePackedGuid(setGuid);
    owner_.getSocket()->send(pkt);
    equipmentSets_.erase(
        std::remove_if(equipmentSets_.begin(), equipmentSets_.end(),
                        [setGuid](const EquipmentSet& es) { return es.setGuid == setGuid; }),
        equipmentSets_.end());
    equipmentSetInfo_.erase(
        std::remove_if(equipmentSetInfo_.begin(), equipmentSetInfo_.end(),
                        [setGuid](const EquipmentSetInfo& info) { return info.setGuid == setGuid; }),
        equipmentSetInfo_.end());
}

void InventoryHandler::handleEquipmentSetList(network::Packet& packet) {
    if (!packet.hasRemaining(4)) return;
    uint32_t count = packet.readUInt32();
    if (count > 10) {
        LOG_WARNING("SMSG_EQUIPMENT_SET_LIST: unexpected count ", count, ", ignoring");
        packet.skipAll();
        return;
    }
    equipmentSets_.clear();
    equipmentSets_.reserve(count);
    for (uint32_t i = 0; i < count; ++i) {
        // Every guid in this message is packed — a mask byte and then only the
        // non-zero bytes — and all twenty were read as fixed eight-byte values.
        // A set guid of one is two bytes on the wire, so the first read ate the
        // set id and the front of the name, and with more than one set the loop
        // lost its place entirely. SendEquipmentSetList writes each of these
        // with appendPackGUID.
        if (!packet.hasRemaining(2)) break;
        EquipmentSet es;
        es.setGuid  = packet.readPackedGuid();
        if (!packet.hasRemaining(4)) break;
        es.setId    = packet.readUInt32();
        es.name     = packet.readString();
        es.iconName = packet.readString();
        // No ignore mask on the wire. A uint32 was being read for one here and
        // the server never sends it: an ignored slot is written as an item guid
        // of literal one, which is the mask. Reading a word that is not there
        // took the first four bytes of the slot list every time.
        es.ignoreSlotMask = 0;
        for (int slot = 0; slot < 19; ++slot) {
            if (!packet.hasRemaining(1)) break;
            const uint64_t itemGuid = packet.readPackedGuid();
            if (itemGuid == 1) {
                // "Ignore this slot" — the set leaves whatever is worn there.
                es.ignoreSlotMask |= (1u << slot);
                es.itemGuids[slot] = 0;
            } else {
                es.itemGuids[slot] = itemGuid;
            }
        }
        equipmentSets_.push_back(std::move(es));
    }
    equipmentSetInfo_.clear();
    equipmentSetInfo_.reserve(equipmentSets_.size());
    for (const auto& es : equipmentSets_) {
        EquipmentSetInfo info;
        info.setGuid  = es.setGuid;
        info.setId    = es.setId;
        info.name     = es.name;
        info.iconName = es.iconName;
        equipmentSetInfo_.push_back(std::move(info));
    }
    LOG_INFO("SMSG_EQUIPMENT_SET_LIST: ", equipmentSets_.size(), " equipment sets received");
}

// ============================================================
// Inventory field / rebuild methods (moved from GameHandler)
// ============================================================

void InventoryHandler::queryItemInfo(uint32_t entry, uint64_t guid) {
    if (owner_.itemInfoCacheRef().count(entry) || owner_.pendingItemQueriesRef().count(entry)) return;
    if (!owner_.isInWorld()) return;

    owner_.pendingItemQueriesRef().insert(entry);
    // Some cores reject CMSG_ITEM_QUERY_SINGLE when the GUID is 0.
    // If we don't have the item object's GUID (e.g. visible equipment decoding),
    // fall back to the player's GUID to keep the request non-zero.
    uint64_t queryGuid = (guid != 0) ? guid : owner_.getPlayerGuid();
    auto packet = owner_.getPacketParsers()
        ? owner_.getPacketParsers()->buildItemQuery(entry, queryGuid)
        : ItemQueryPacket::build(entry, queryGuid);
    owner_.getSocket()->send(packet);
    LOG_DEBUG("queryItemInfo: SENT entry=", entry, " guid=0x", std::hex, queryGuid, std::dec,
              " pending=", owner_.pendingItemQueriesRef().size());
}

void InventoryHandler::handleItemQueryResponse(network::Packet& packet) {
    ItemQueryResponseData data;
    bool parsed = owner_.getPacketParsers()
        ? owner_.getPacketParsers()->parseItemQueryResponse(packet, data)
        : ItemQueryResponseParser::parse(packet, data);
    if (!parsed) {
        // Extract entry from raw packet so we can clear the pending query even on parse failure.
        // Without this, the entry stays in pendingItemQueries_ forever, blocking retries.
        if (packet.getSize() >= 4) {
            packet.setReadPos(0);
            // High bit indicates a negative (invalid/missing) item entry response;
            // mask it off so we can still clear the pending query by entry ID.
            uint32_t rawEntry = packet.readUInt32() & ~0x80000000u;
            owner_.pendingItemQueriesRef().erase(rawEntry);
        }
        LOG_WARNING("handleItemQueryResponse: parse failed, size=", packet.getSize());
        return;
    }

    owner_.pendingItemQueriesRef().erase(data.entry);
    LOG_DEBUG("handleItemQueryResponse: RECV entry=", data.entry, " name='", data.name,
              "' displayInfoId=", data.displayInfoId,
              " class=", data.itemClass, " subClass=", data.subClass,
              " invType=", data.inventoryType, " valid=", data.valid);

    if (data.valid) {
        owner_.itemInfoCacheRef()[data.entry] = data;
        rebuildOnlineInventory();
        maybeDetectVisibleItemLayout();

        // Auction mail subjects contain an item entry rather than display text.
        // Refresh FrameXML mail rows once that item's name becomes available.
        bool resolvedAuctionSubject = false;
        for (const auto& mail : mailInbox_) {
            AuctionMailSubject auction;
            if (mail.messageType == 2 && parseAuctionMailSubject(mail.subject, auction) &&
                auction.itemEntry == data.entry) {
                resolvedAuctionSubject = true;
                break;
            }
        }
        if (resolvedAuctionSubject && owner_.addonEventCallbackRef()) {
            owner_.addonEventCallbackRef()("MAIL_INBOX_UPDATE", {});
        }

        // Flush any deferred loot notifications waiting on this item's name/quality.
        for (auto it = owner_.pendingItemPushNotifsRef().begin(); it != owner_.pendingItemPushNotifsRef().end(); ) {
            if (it->itemId == data.entry) {
                std::string itemName = data.name.empty() ? ("item #" + std::to_string(data.entry)) : data.name;
                std::string link = buildItemLink(data.entry, data.quality, itemName);
                std::string msg = "Received item: " + link;
                if (it->count > 1) msg += " x" + std::to_string(it->count);
                owner_.addSystemChatMessage(msg);
                if (auto* ac = owner_.services().audioCoordinator) {
                    if (auto* sfx = ac->getUiSoundManager()) sfx->playLootItem();
                }
                if (owner_.itemLootCallbackRef()) owner_.itemLootCallbackRef()(data.entry, it->count, data.quality, itemName);
                it = owner_.pendingItemPushNotifsRef().erase(it);
            } else {
                ++it;
            }
        }

        // Selectively re-emit only players whose equipment references this item entry
        const uint32_t resolvedEntry = data.entry;
        int reemitCount = 0;
        for (const auto& [guid, entries] : owner_.otherPlayerVisibleItemEntriesRef()) {
            for (uint32_t e : entries) {
                if (e == resolvedEntry) {
                    emitOtherPlayerEquipment(guid);
                    reemitCount++;
                    break;
                }
            }
        }
        if (reemitCount > 0) {
            LOG_DEBUG("Re-emitted equipment for ", reemitCount, " players after resolving entry=", resolvedEntry);
        }
        // Same for inspect-based entries
        if (owner_.playerEquipmentCallbackRef()) {
            for (const auto& [guid, entries] : owner_.inspectedPlayerItemEntriesRef()) {
                bool relevant = false;
                for (uint32_t e : entries) {
                    if (e == resolvedEntry) { relevant = true; break; }
                }
                if (!relevant) continue;
                std::array<uint32_t, 19> displayIds{};
                std::array<uint8_t, 19> invTypes{};
                for (int s = 0; s < 19; s++) {
                    uint32_t entry = entries[s];
                    if (entry == 0) continue;
                    auto infoIt = owner_.itemInfoCacheRef().find(entry);
                    if (infoIt == owner_.itemInfoCacheRef().end()) continue;
                    displayIds[s] = infoIt->second.displayInfoId;
                    invTypes[s] = static_cast<uint8_t>(infoIt->second.inventoryType);
                }
                owner_.playerEquipmentCallbackRef()(guid, displayIds, invTypes);
            }
        }
    }
}

uint64_t InventoryHandler::resolveOnlineItemGuid(uint32_t itemId) const {
    if (itemId == 0) return 0;
    uint64_t candidate = 0;
    for (const auto& [guid, info] : owner_.onlineItemsRef()) {
        if (info.entry != itemId) continue;
        // Item IDs are templates, not identities. Only use this compatibility
        // fallback when the matching online object is unambiguous.
        if (candidate != 0) return 0;
        candidate = guid;
    }
    return candidate;
}

void InventoryHandler::detectInventorySlotBases(const FlatFieldMap& fields) {
    if (owner_.invSlotBaseRef() >= 0 && owner_.packSlotBaseRef() >= 0) return;
    if (fields.empty()) return;

    std::vector<uint16_t> matchingPairs;
    matchingPairs.reserve(32);

    for (const auto& [idx, low] : fields) {
        if ((idx % 2) != 0) continue;
        auto itHigh = fields.find(static_cast<uint16_t>(idx + 1));
        if (itHigh == fields.end()) continue;
        uint64_t guid = (uint64_t(itHigh->second) << 32) | low;
        if (guid == 0) continue;
        // Primary signal: GUID pairs that match spawned ITEM objects.
        if (!owner_.onlineItemsRef().empty() && owner_.onlineItemsRef().count(guid)) {
            matchingPairs.push_back(idx);
        }
    }

    // Fallback signal (when ITEM objects haven't been seen yet):
    // collect any plausible non-zero GUID pairs and derive a base by density.
    if (matchingPairs.empty()) {
        for (const auto& [idx, low] : fields) {
            if ((idx % 2) != 0) continue;
            auto itHigh = fields.find(static_cast<uint16_t>(idx + 1));
            if (itHigh == fields.end()) continue;
            uint64_t guid = (uint64_t(itHigh->second) << 32) | low;
            if (guid == 0) continue;
            // Heuristic: item GUIDs tend to be non-trivial and change often; ignore tiny values.
            if (guid < 0x10000ull) continue;
            matchingPairs.push_back(idx);
        }
    }

    if (matchingPairs.empty()) return;
    std::sort(matchingPairs.begin(), matchingPairs.end());

    if (owner_.invSlotBaseRef() < 0) {
        // The lowest matching field is the first EQUIPPED slot (not necessarily HEAD).
        // With 2+ matches we can derive the true base: all matches must be at
        // even offsets from the base, spaced 2 fields per slot.
        const int knownBase = static_cast<int>(fieldIndex(UF::PLAYER_FIELD_INV_SLOT_HEAD));
        constexpr int slotStride = 2;
        bool allAlign = true;
        for (uint16_t p : matchingPairs) {
            if (p < knownBase || (p - knownBase) % slotStride != 0) {
                allAlign = false;
                break;
            }
        }
        if (allAlign) {
            owner_.invSlotBaseRef() = knownBase;
        } else {
            // Fallback: if we have 2+ matches, derive base from their spacing
            if (matchingPairs.size() >= 2) {
                uint16_t lo = matchingPairs[0];
                // lo must be base + 2*slotN, and slotN is 0..22
                // Try each possible slot for 'lo' and see if all others also land on valid slots
                for (int s = 0; s <= 22; s++) {
                    int candidate = lo - s * slotStride;
                    if (candidate < 0) break;
                    bool ok = true;
                    for (uint16_t p : matchingPairs) {
                        int off = p - candidate;
                        if (off < 0 || off % slotStride != 0 || off / slotStride > 22) {
                            ok = false;
                            break;
                        }
                    }
                    if (ok) {
                        owner_.invSlotBaseRef() = candidate;
                        break;
                    }
                }
                if (owner_.invSlotBaseRef() < 0) owner_.invSlotBaseRef() = knownBase;
            } else {
                owner_.invSlotBaseRef() = knownBase;
            }
        }
        owner_.packSlotBaseRef() = owner_.invSlotBaseRef() + (game::Inventory::NUM_EQUIP_SLOTS * 2);
        LOG_INFO("Detected inventory field base: equip=", owner_.invSlotBaseRef(),
                 " pack=", owner_.packSlotBaseRef());
    }
}

bool InventoryHandler::applyInventoryFields(const FlatFieldMap& fields) {
    bool slotsChanged = false;
    bool buybackSlotsChanged = false;
    int equipBase = (owner_.invSlotBaseRef() >= 0) ? owner_.invSlotBaseRef() : static_cast<int>(fieldIndex(UF::PLAYER_FIELD_INV_SLOT_HEAD));
    int packBase = (owner_.packSlotBaseRef() >= 0) ? owner_.packSlotBaseRef() : static_cast<int>(fieldIndex(UF::PLAYER_FIELD_PACK_SLOT_1));
    int bankBase = static_cast<int>(fieldIndex(UF::PLAYER_FIELD_BANK_SLOT_1));
    int bankBagBase = static_cast<int>(fieldIndex(UF::PLAYER_FIELD_BANKBAG_SLOT_1));

    // Derive slot counts from field gap (Classic=24/6, TBC/WotLK=28/7).
    if (bankBase != 0xFFFF && bankBagBase != 0xFFFF) {
        effectiveBankSlots_ = std::min((bankBagBase - bankBase) / 2, 28);
        effectiveBankBagSlots_ = (effectiveBankSlots_ <= 24) ? 6 : 7;
    }

    int keyringBase = static_cast<int>(fieldIndex(UF::PLAYER_FIELD_KEYRING_SLOT_1));
    const int buybackBase = bankBagBase != 0xFFFF
        ? bankBagBase + (effectiveBankBagSlots_ * 2) : 0xFFFF;
    if (keyringBase == 0xFFFF && bankBagBase != 0xFFFF) {
        // Layout fallback for profiles that don't define PLAYER_FIELD_KEYRING_SLOT_1.
        // Bank bag slots are followed by 12 vendor buyback slots (24 fields), then keyring.
        keyringBase = bankBagBase + (effectiveBankBagSlots_ * 2) + 24;
    }

    for (const auto& [key, val] : fields) {
        if (key >= equipBase && key <= equipBase + (game::Inventory::NUM_EQUIP_SLOTS * 2 - 1)) {
            int slotIndex = (key - equipBase) / 2;
            bool isLow = ((key - equipBase) % 2 == 0);
            if (slotIndex < static_cast<int>(owner_.equipSlotGuidsRef().size())) {
                uint64_t& guid = owner_.equipSlotGuidsRef()[slotIndex];
                if (isLow) guid = (guid & 0xFFFFFFFF00000000ULL) | val;
                else guid = (guid & 0x00000000FFFFFFFFULL) | (uint64_t(val) << 32);
                slotsChanged = true;
            }
        } else if (key >= packBase && key <= packBase + (game::Inventory::BACKPACK_SLOTS * 2 - 1)) {
            int slotIndex = (key - packBase) / 2;
            bool isLow = ((key - packBase) % 2 == 0);
            if (slotIndex < static_cast<int>(owner_.backpackSlotGuidsRef().size())) {
                uint64_t& guid = owner_.backpackSlotGuidsRef()[slotIndex];
                if (isLow) guid = (guid & 0xFFFFFFFF00000000ULL) | val;
                else guid = (guid & 0x00000000FFFFFFFFULL) | (uint64_t(val) << 32);
                slotsChanged = true;
            }
        } else if (keyringBase != 0xFFFF &&
                   key >= keyringBase &&
                   key <= keyringBase + (game::Inventory::KEYRING_SLOTS * 2 - 1)) {
            int slotIndex = (key - keyringBase) / 2;
            bool isLow = ((key - keyringBase) % 2 == 0);
            if (slotIndex < static_cast<int>(owner_.keyringSlotGuidsRef().size())) {
                uint64_t& guid = owner_.keyringSlotGuidsRef()[slotIndex];
                if (isLow) guid = (guid & 0xFFFFFFFF00000000ULL) | val;
                else guid = (guid & 0x00000000FFFFFFFFULL) | (uint64_t(val) << 32);
                slotsChanged = true;
            }
        }
        if (bankBase != 0xFFFF && key >= static_cast<uint16_t>(bankBase) &&
            key <= static_cast<uint16_t>(bankBase) + (effectiveBankSlots_ * 2 - 1)) {
            int slotIndex = (key - bankBase) / 2;
            bool isLow = ((key - bankBase) % 2 == 0);
            if (slotIndex < static_cast<int>(bankSlotGuids_.size())) {
                uint64_t& guid = bankSlotGuids_[slotIndex];
                if (isLow) guid = (guid & 0xFFFFFFFF00000000ULL) | val;
                else guid = (guid & 0x00000000FFFFFFFFULL) | (uint64_t(val) << 32);
                slotsChanged = true;
            }
        }

        // Bank bag slots starting at PLAYER_FIELD_BANKBAG_SLOT_1
        if (bankBagBase != 0xFFFF && key >= static_cast<uint16_t>(bankBagBase) &&
            key <= static_cast<uint16_t>(bankBagBase) + (effectiveBankBagSlots_ * 2 - 1)) {
            int slotIndex = (key - bankBagBase) / 2;
            bool isLow = ((key - bankBagBase) % 2 == 0);
            if (slotIndex < static_cast<int>(bankBagSlotGuids_.size())) {
                uint64_t& guid = bankBagSlotGuids_[slotIndex];
                if (isLow) guid = (guid & 0xFFFFFFFF00000000ULL) | val;
                else guid = (guid & 0x00000000FFFFFFFFULL) | (uint64_t(val) << 32);
                slotsChanged = true;
            }
        }

        // The server owns the buyback ring. Bind local item metadata to these
        // GUID fields rather than assuming displayed row N means wire slot 74+N.
        if (buybackBase != 0xFFFF && key >= static_cast<uint16_t>(buybackBase) &&
            key <= static_cast<uint16_t>(buybackBase) +
                   (kBuybackWireSlotCount * 2 - 1)) {
            const int slotIndex = (key - buybackBase) / 2;
            const bool isLow = ((key - buybackBase) % 2 == 0);
            uint64_t& guid = buybackSlotGuids_[slotIndex];
            if (isLow) guid = (guid & 0xFFFFFFFF00000000ULL) | val;
            else guid = (guid & 0x00000000FFFFFFFFULL) | (uint64_t(val) << 32);
            buybackSlotsChanged = true;
        }
    }

    if (buybackSlotsChanged) reconcileBuybackSlots();

    return slotsChanged;
}

void InventoryHandler::extractContainerFields(uint64_t containerGuid, const FlatFieldMap& fields) {
    const uint16_t numSlotsIdx = fieldIndex(UF::CONTAINER_FIELD_NUM_SLOTS);
    const uint16_t slot1Idx = fieldIndex(UF::CONTAINER_FIELD_SLOT_1);
    if (numSlotsIdx == 0xFFFF || slot1Idx == 0xFFFF) return;

    auto& info = owner_.containerContentsRef()[containerGuid];

    // Read number of slots
    auto numIt = fields.find(numSlotsIdx);
    if (numIt != fields.end()) {
        info.numSlots = std::min(numIt->second, 36u);
    }

    // Read slot GUIDs (each is 2 uint32 fields: lo + hi)
    for (const auto& [key, val] : fields) {
        if (key < slot1Idx) continue;
        int offset = key - slot1Idx;
        int slotIndex = offset / 2;
        if (slotIndex >= 36) continue;
        bool isLow = (offset % 2 == 0);
        uint64_t& guid = info.slotGuids[slotIndex];
        if (isLow) guid = (guid & 0xFFFFFFFF00000000ULL) | val;
        else guid = (guid & 0x00000000FFFFFFFFULL) | (uint64_t(val) << 32);
    }
}

// Builds an ItemDef from an OnlineItemInfo by merging server field data (stack
// count, durability) with cached item info (name, stats, quality). Centralised
// here so adding new ItemDef fields doesn't require editing 4 separate copy-
// paste sites in rebuildOnlineInventory.
ItemDef InventoryHandler::buildItemDef(uint32_t entry, uint32_t stackCount,
                                       uint32_t curDur, uint32_t maxDur, uint64_t guid,
                                       uint32_t flags, int32_t randomPropertyId,
                                       uint32_t suffixFactor) {
    ItemDef def;
    def.itemId = entry;
    def.guid = guid;
    def.stackCount = stackCount;
    def.curDurability = curDur;
    def.maxDurability = maxDur;
    def.maxStack = 1;
    // ITEM_FLAG_SOULBOUND (0x1): a BoE item that has already bound must not re-prompt.
    def.soulbound = (flags & 0x1u) != 0;
    def.randomPropertyId = randomPropertyId;

    auto infoIt = owner_.itemInfoCacheRef().find(entry);
    if (infoIt != owner_.itemInfoCacheRef().end()) {
        const auto& info = infoIt->second;
        def.name = info.name;
        def.quality = static_cast<ItemQuality>(info.quality);
        def.inventoryType = info.inventoryType;
        def.maxStack = std::max(1, info.maxStack);
        def.displayInfoId = info.displayInfoId;
        def.subclassName = info.subclassName;
        def.damageMin = info.damageMin;
        def.damageMax = info.damageMax;
        def.delayMs = info.delayMs;
        def.armor = info.armor;
        def.stamina = info.stamina;
        def.strength = info.strength;
        def.agility = info.agility;
        def.intellect = info.intellect;
        def.spirit = info.spirit;
        def.sellPrice = info.sellPrice;
        def.itemLevel = info.itemLevel;
        def.requiredLevel = info.requiredLevel;
        def.bindType = info.bindType;
        def.description = info.description;
        def.pageTextId = info.pageTextId;
        def.startQuestId = info.startQuestId;
        def.extraStats.clear();
        for (const auto& es : info.extraStats)
            def.extraStats.push_back({es.statType, es.statValue});
    } else {
        def.name = "Item " + std::to_string(def.itemId);
        queryItemInfo(def.itemId, guid);
    }

    // Merge the rolled random-suffix/property stats on top of the base template. Primary
    // stats fold into their dedicated fields; everything else (ratings, AP, spell power)
    // joins extraStats so the existing tooltip rendering shows them as green stat lines.
    if (randomPropertyId != 0) {
        for (const auto& b : owner_.getRandomStatBonuses(randomPropertyId, suffixFactor)) {
            if (b.value == 0) continue;
            switch (b.statType) {
                case 3: def.agility   += b.value; break;
                case 4: def.strength  += b.value; break;
                case 5: def.intellect += b.value; break;
                case 6: def.spirit    += b.value; break;
                case 7: def.stamina   += b.value; break;
                default: def.extraStats.push_back({b.statType, b.value}); break;
            }
        }
    }
    return def;
}

void InventoryHandler::rebuildOnlineInventory() {

    uint8_t savedBankBagSlots = owner_.inventoryRef().getPurchasedBankBagSlots();
    owner_.inventoryRef() = Inventory();
    owner_.inventoryRef().setPurchasedBankBagSlots(savedBankBagSlots);

    // Equipment slots
    for (int i = 0; i < 23; i++) {
        uint64_t guid = owner_.equipSlotGuidsRef()[i];
        if (guid == 0) continue;
        auto itemIt = owner_.onlineItemsRef().find(guid);
        if (itemIt == owner_.onlineItemsRef().end()) continue;
        owner_.inventoryRef().setEquipSlot(static_cast<EquipSlot>(i), buildItemDef(itemIt->second.entry, itemIt->second.stackCount, itemIt->second.curDurability, itemIt->second.maxDurability, guid, itemIt->second.flags, itemIt->second.randomPropertyId, itemIt->second.suffixFactor));
    }

    // Backpack slots
    for (int i = 0; i < 16; i++) {
        uint64_t guid = owner_.backpackSlotGuidsRef()[i];
        if (guid == 0) continue;
        auto itemIt = owner_.onlineItemsRef().find(guid);
        if (itemIt == owner_.onlineItemsRef().end()) continue;
        owner_.inventoryRef().setBackpackSlot(i, buildItemDef(itemIt->second.entry, itemIt->second.stackCount, itemIt->second.curDurability, itemIt->second.maxDurability, guid, itemIt->second.flags, itemIt->second.randomPropertyId, itemIt->second.suffixFactor));
    }

    // Keyring slots
    for (int i = 0; i < game::Inventory::KEYRING_SLOTS; i++) {
        uint64_t guid = owner_.keyringSlotGuidsRef()[i];
        if (guid == 0) continue;
        auto itemIt = owner_.onlineItemsRef().find(guid);
        if (itemIt == owner_.onlineItemsRef().end()) continue;
        owner_.inventoryRef().setKeyringSlot(i, buildItemDef(itemIt->second.entry, itemIt->second.stackCount, itemIt->second.curDurability, itemIt->second.maxDurability, guid, itemIt->second.flags, itemIt->second.randomPropertyId, itemIt->second.suffixFactor));
    }

    // Bag contents (BAG1-BAG4 are equip slots 19-22)
    for (int bagIdx = 0; bagIdx < 4; bagIdx++) {
        uint64_t bagGuid = owner_.equipSlotGuidsRef()[Inventory::FIRST_BAG_EQUIP_SLOT + bagIdx];
        if (bagGuid == 0) continue;

        // Determine bag size from container fields or item template
        int numSlots = 0;
        auto contIt = owner_.containerContentsRef().find(bagGuid);
        if (contIt != owner_.containerContentsRef().end()) {
            numSlots = static_cast<int>(contIt->second.numSlots);
        }
        const ItemQueryResponseData* bagTemplate = nullptr;
        {
            auto bagItemIt = owner_.onlineItemsRef().find(bagGuid);
            if (bagItemIt != owner_.onlineItemsRef().end()) {
                auto bagInfoIt = owner_.itemInfoCacheRef().find(bagItemIt->second.entry);
                if (bagInfoIt != owner_.itemInfoCacheRef().end())
                    bagTemplate = &bagInfoIt->second;
            }
        }
        if (numSlots <= 0 && bagTemplate) {
            numSlots = bagTemplate->containerSlots;
        }
        if (numSlots <= 0) continue;

        // Set the bag size in the inventory bag data
        owner_.inventoryRef().setBagSize(bagIdx, numSlots);
        // Quivers (class 11) and profession bags (class 1, subclass != 0) only
        // accept their own item type — sorting and the combined grid must know.
        owner_.inventoryRef().setBagSpecial(bagIdx, bagTemplate &&
            (bagTemplate->itemClass == 11 ||
             (bagTemplate->itemClass == 1 && bagTemplate->subClass != 0)));

        // Also set bagSlots on the equipped bag item (for UI display)
        auto& bagEquipSlot = owner_.inventoryRef().getEquipSlot(static_cast<EquipSlot>(Inventory::FIRST_BAG_EQUIP_SLOT + bagIdx));
        if (!bagEquipSlot.empty()) {
            ItemDef bagDef = bagEquipSlot.item;
            bagDef.bagSlots = numSlots;
            owner_.inventoryRef().setEquipSlot(static_cast<EquipSlot>(Inventory::FIRST_BAG_EQUIP_SLOT + bagIdx), bagDef);
        }

        // Populate bag slot items
        if (contIt == owner_.containerContentsRef().end()) continue;
        const auto& container = contIt->second;
        for (int s = 0; s < numSlots && s < 36; s++) {
            uint64_t itemGuid = container.slotGuids[s];
            if (itemGuid == 0) continue;

            auto itemIt = owner_.onlineItemsRef().find(itemGuid);
            if (itemIt == owner_.onlineItemsRef().end()) continue;
            ItemDef def = buildItemDef(itemIt->second.entry, itemIt->second.stackCount, itemIt->second.curDurability, itemIt->second.maxDurability, itemGuid, itemIt->second.flags, itemIt->second.randomPropertyId, itemIt->second.suffixFactor);
            // Bags inside bags need containerSlots for the UI slot-count display.
            auto bagInfoIt = owner_.itemInfoCacheRef().find(itemIt->second.entry);
            if (bagInfoIt != owner_.itemInfoCacheRef().end())
                def.bagSlots = bagInfoIt->second.containerSlots;
            owner_.inventoryRef().setBagSlot(bagIdx, s, def);
        }
    }

    // Bank slots (24 for Classic, 28 for TBC/WotLK)
    for (int i = 0; i < effectiveBankSlots_; i++) {
        uint64_t guid = bankSlotGuids_[i];
        if (guid == 0) { owner_.inventoryRef().clearBankSlot(i); continue; }

        auto itemIt = owner_.onlineItemsRef().find(guid);
        if (itemIt == owner_.onlineItemsRef().end()) { owner_.inventoryRef().clearBankSlot(i); continue; }

        ItemDef def;
        def.itemId = itemIt->second.entry;
        def.stackCount = itemIt->second.stackCount;
        def.curDurability = itemIt->second.curDurability;
        def.maxDurability = itemIt->second.maxDurability;
        def.maxStack = 1;

        auto infoIt = owner_.itemInfoCacheRef().find(itemIt->second.entry);
        if (infoIt != owner_.itemInfoCacheRef().end()) {
            def.name = infoIt->second.name;
            def.quality = static_cast<ItemQuality>(infoIt->second.quality);
            def.inventoryType = infoIt->second.inventoryType;
            def.maxStack = std::max(1, infoIt->second.maxStack);
            def.displayInfoId = infoIt->second.displayInfoId;
            def.subclassName = infoIt->second.subclassName;
            def.damageMin = infoIt->second.damageMin;
            def.damageMax = infoIt->second.damageMax;
            def.delayMs = infoIt->second.delayMs;
            def.armor = infoIt->second.armor;
            def.stamina = infoIt->second.stamina;
            def.strength = infoIt->second.strength;
            def.agility = infoIt->second.agility;
            def.intellect = infoIt->second.intellect;
            def.spirit = infoIt->second.spirit;
            def.itemLevel = infoIt->second.itemLevel;
            def.requiredLevel = infoIt->second.requiredLevel;
            def.bindType = infoIt->second.bindType;
            def.description = infoIt->second.description;
            def.startQuestId = infoIt->second.startQuestId;
            def.extraStats.clear();
            for (const auto& es : infoIt->second.extraStats)
                def.extraStats.push_back({es.statType, es.statValue});
            def.sellPrice = infoIt->second.sellPrice;
            def.bagSlots = infoIt->second.containerSlots;
        } else {
            def.name = "Item " + std::to_string(def.itemId);
            queryItemInfo(def.itemId, guid);
        }

        owner_.inventoryRef().setBankSlot(i, def);
    }

    // Bank bag contents (6 for Classic, 7 for TBC/WotLK)
    for (int bagIdx = 0; bagIdx < effectiveBankBagSlots_; bagIdx++) {
        uint64_t bagGuid = bankBagSlotGuids_[bagIdx];
        if (bagGuid == 0) { owner_.inventoryRef().setBankBagSize(bagIdx, 0); continue; }

        int numSlots = 0;
        auto contIt = owner_.containerContentsRef().find(bagGuid);
        if (contIt != owner_.containerContentsRef().end()) {
            numSlots = static_cast<int>(contIt->second.numSlots);
        }

        // Populate the bag item itself (for icon/name in the bank bag equip slot)
        auto bagItemIt = owner_.onlineItemsRef().find(bagGuid);
        if (bagItemIt != owner_.onlineItemsRef().end()) {
            if (numSlots <= 0) {
                auto bagInfoIt = owner_.itemInfoCacheRef().find(bagItemIt->second.entry);
                if (bagInfoIt != owner_.itemInfoCacheRef().end()) {
                    numSlots = bagInfoIt->second.containerSlots;
                }
            }
            ItemDef bagDef;
            bagDef.itemId = bagItemIt->second.entry;
            bagDef.stackCount = 1;
            bagDef.inventoryType = 18; // bag
            auto bagInfoIt = owner_.itemInfoCacheRef().find(bagItemIt->second.entry);
            if (bagInfoIt != owner_.itemInfoCacheRef().end()) {
                bagDef.name = bagInfoIt->second.name;
                bagDef.quality = static_cast<ItemQuality>(bagInfoIt->second.quality);
                bagDef.displayInfoId = bagInfoIt->second.displayInfoId;
                bagDef.bagSlots = bagInfoIt->second.containerSlots;
            } else {
                bagDef.name = "Bag";
                queryItemInfo(bagDef.itemId, bagGuid);
            }
            owner_.inventoryRef().setBankBagItem(bagIdx, bagDef);
        }
        if (numSlots <= 0) continue;

        owner_.inventoryRef().setBankBagSize(bagIdx, numSlots);

        if (contIt == owner_.containerContentsRef().end()) continue;
        const auto& container = contIt->second;
        for (int s = 0; s < numSlots && s < 36; s++) {
            uint64_t itemGuid = container.slotGuids[s];
            if (itemGuid == 0) continue;

            auto itemIt = owner_.onlineItemsRef().find(itemGuid);
            if (itemIt == owner_.onlineItemsRef().end()) continue;

            ItemDef def;
            def.itemId = itemIt->second.entry;
            def.stackCount = itemIt->second.stackCount;
        def.curDurability = itemIt->second.curDurability;
        def.maxDurability = itemIt->second.maxDurability;
            def.maxStack = 1;

            auto infoIt = owner_.itemInfoCacheRef().find(itemIt->second.entry);
            if (infoIt != owner_.itemInfoCacheRef().end()) {
                def.name = infoIt->second.name;
                def.quality = static_cast<ItemQuality>(infoIt->second.quality);
                def.inventoryType = infoIt->second.inventoryType;
                def.maxStack = std::max(1, infoIt->second.maxStack);
                def.displayInfoId = infoIt->second.displayInfoId;
                def.subclassName = infoIt->second.subclassName;
                def.damageMin = infoIt->second.damageMin;
                def.damageMax = infoIt->second.damageMax;
                def.delayMs = infoIt->second.delayMs;
                def.armor = infoIt->second.armor;
                def.stamina = infoIt->second.stamina;
                def.strength = infoIt->second.strength;
                def.agility = infoIt->second.agility;
                def.intellect = infoIt->second.intellect;
                def.spirit = infoIt->second.spirit;
                def.itemLevel = infoIt->second.itemLevel;
                def.requiredLevel = infoIt->second.requiredLevel;
                def.sellPrice = infoIt->second.sellPrice;
                def.bindType = infoIt->second.bindType;
                def.description = infoIt->second.description;
                def.startQuestId = infoIt->second.startQuestId;
                def.extraStats.clear();
                for (const auto& es : infoIt->second.extraStats)
                    def.extraStats.push_back({es.statType, es.statValue});
                def.bagSlots = infoIt->second.containerSlots;
            } else {
                def.name = "Item " + std::to_string(def.itemId);
                queryItemInfo(def.itemId, itemGuid);
            }

            owner_.inventoryRef().setBankBagSlot(bagIdx, s, def);
        }
    }

    // Only mark equipment dirty if equipped item displayInfoIds actually changed.
    // Enchants count too: applying a sharpening stone leaves the displayInfoId alone
    // but changes the visual effect hanging off the weapon.
    std::array<uint32_t, 19> currentEquipDisplayIds{};
    std::array<uint64_t, 19> currentEquipEnchantIds{};
    for (int i = 0; i < 19; i++) {
        const auto& slot = owner_.inventoryRef().getEquipSlot(static_cast<EquipSlot>(i));
        if (!slot.empty()) currentEquipDisplayIds[i] = slot.item.displayInfoId;

        uint64_t guid = owner_.equipSlotGuidsRef()[i];
        if (guid != 0) {
            auto [permEnchantId, tempEnchantId] = owner_.getItemEnchantIds(guid);
            currentEquipEnchantIds[i] =
                (static_cast<uint64_t>(permEnchantId) << 32) | tempEnchantId;
        }
    }
    if (currentEquipDisplayIds != owner_.lastEquipDisplayIdsRef() ||
        currentEquipEnchantIds != lastEquipEnchantIds_) {
        owner_.lastEquipDisplayIdsRef() = currentEquipDisplayIds;
        lastEquipEnchantIds_ = currentEquipEnchantIds;
        owner_.onlineEquipDirtyRef() = true;
    }

    LOG_DEBUG("Rebuilt online inventory: equip=", [&](){
        int c = 0; for (auto g : owner_.equipSlotGuidsRef()) if (g) c++; return c;
    }(), " backpack=", [&](){
        int c = 0; for (auto g : owner_.backpackSlotGuidsRef()) if (g) c++; return c;
    }(), " keyring=", [&](){
        int c = 0; for (auto g : owner_.keyringSlotGuidsRef()) if (g) c++; return c;
    }());

    // Reconcile collect-item quest objectives against what the player is
    // actually carrying. In 3.3.5a the server never pushes item objective
    // counts, so this bag-count pass is the only thing that advances "collect
    // N of item" progress when quest items are looted (or removed). Count
    // backpack + the four equipped bags — the same set the server checks at
    // turn-in — summing stacks per item id.
    std::unordered_map<uint32_t, uint32_t> carriedCounts;
    const auto& inv = owner_.inventoryRef();
    for (int i = 0; i < inv.getBackpackSize(); i++) {
        const auto& slot = inv.getBackpackSlot(i);
        if (!slot.empty())
            carriedCounts[slot.item.itemId] += std::max<uint32_t>(1, slot.item.stackCount);
    }
    for (int bagIdx = 0; bagIdx < 4; bagIdx++) {
        int numSlots = inv.getBagSize(bagIdx);
        for (int s = 0; s < numSlots; s++) {
            const auto& slot = inv.getBagSlot(bagIdx, s);
            if (!slot.empty())
                carriedCounts[slot.item.itemId] += std::max<uint32_t>(1, slot.item.stackCount);
        }
    }
    owner_.reconcileQuestItemObjectives(carriedCounts);
}

void InventoryHandler::maybeDetectVisibleItemLayout() {
    if (owner_.visibleItemLayoutVerifiedRef()) return;
    if (usesVisibleItemDisplayIds()) {
        std::array<uint32_t, 19> equipDisplayIds = owner_.lastEquipDisplayIdsRef();
        int nonZero = 0;
        for (int i = 0; i < 19; ++i) {
            if (equipDisplayIds[i] == 0) {
                const auto& slot = owner_.inventoryRef().getEquipSlot(static_cast<EquipSlot>(i));
                if (!slot.empty()) equipDisplayIds[i] = slot.item.displayInfoId;
            }
            if (equipDisplayIds[i] != 0) nonZero++;
        }

        int bestBase = -1;
        int bestStride = 0;
        int bestMatches = 0;
        int bestMismatches = 9999;
        int bestScore = -999999;

        if (nonZero >= 2 && !owner_.lastPlayerFieldsRef().empty()) {
            const uint16_t maxKey = owner_.lastPlayerFieldsRef().back().first;
            const int strides[] = {16, 12, 8, 4, 2, 3, 1};
            for (int stride : strides) {
                for (const auto& [baseIdxU16, _v] : owner_.lastPlayerFieldsRef()) {
                    const int base = static_cast<int>(baseIdxU16);
                    if (base + 18 * stride > static_cast<int>(maxKey)) continue;

                    int matches = 0;
                    int mismatches = 0;
                    for (int s = 0; s < 19; ++s) {
                        uint32_t want = equipDisplayIds[s];
                        if (want == 0) continue;
                        const uint16_t idx = static_cast<uint16_t>(base + s * stride);
                        auto it = owner_.lastPlayerFieldsRef().find(idx);
                        if (it == owner_.lastPlayerFieldsRef().end()) continue;
                        if (it->second == want) {
                            matches++;
                        } else if (it->second != 0) {
                            mismatches++;
                        }
                    }

                    int score = matches * 3 - mismatches * 2;
                    if (score > bestScore ||
                        (score == bestScore && matches > bestMatches) ||
                        (score == bestScore && matches == bestMatches && mismatches < bestMismatches) ||
                        (score == bestScore && matches == bestMatches && mismatches == bestMismatches &&
                         (bestBase < 0 || base < bestBase))) {
                        bestScore = score;
                        bestMatches = matches;
                        bestMismatches = mismatches;
                        bestBase = base;
                        bestStride = stride;
                    }
                }
            }
        }

        bool changed = false;
        if (bestMatches >= 2 && bestBase >= 0 && bestStride > 0 && bestMismatches <= 2) {
            changed = owner_.visibleItemEntryBaseRef() != bestBase ||
                      owner_.visibleItemStrideRef() != bestStride;
            owner_.visibleItemEntryBaseRef() = bestBase;
            owner_.visibleItemStrideRef() = bestStride;
            owner_.visibleItemLayoutVerifiedRef() = true;
            LOG_INFO("Detected TBC PLAYER_VISIBLE_ITEM display layout: base=", bestBase,
                     " stride=", bestStride, " (matches=", bestMatches,
                     " mismatches=", bestMismatches, " score=", bestScore, ")");
        } else {
            // TBC exposes display IDs rather than item entries, but private cores
            // are not perfectly consistent about the base offset. Keep the known
            // 2.4.x fallback active without marking it verified, so later local
            // equipment updates can still detect a better layout.
            const int kTbcFallbackBase = tbcVisibleItemBaseFallback();
            constexpr int kTbcFallbackStride = kTbcVisibleItemStride;
            changed = owner_.visibleItemEntryBaseRef() != kTbcFallbackBase ||
                      owner_.visibleItemStrideRef() != kTbcFallbackStride;
            owner_.visibleItemEntryBaseRef() = kTbcFallbackBase;
            owner_.visibleItemStrideRef() = kTbcFallbackStride;
            static bool loggedProvisionalLayout = false;
            if (!loggedProvisionalLayout) {
                loggedProvisionalLayout = true;
                LOG_INFO("Using provisional TBC PLAYER_VISIBLE_ITEM display layout: base=",
                         kTbcFallbackBase, " stride=", kTbcFallbackStride,
                         " (waiting for local display-ID confirmation)");
            }
            if (visibleEquipmentFieldDiagEnabled()) {
                LOG_INFO("TBC visible layout detection pending: localDisplayIds=", nonZero,
                         " bestBase=", bestBase, " bestStride=", bestStride,
                         " matches=", bestMatches, " mismatches=", bestMismatches,
                         " score=", bestScore);
            }
        }

        if (changed || owner_.visibleItemLayoutVerifiedRef()) {
            for (const auto& [guid, ent] : owner_.getEntityManager().getEntities()) {
                if (!ent || ent->getType() != ObjectType::PLAYER) continue;
                if (guid == owner_.getPlayerGuid()) continue;
                updateOtherPlayerVisibleItems(guid, ent->getFields());
            }
        }
        return;
    }
    if (owner_.lastPlayerFieldsRef().empty()) return;

    if (isActiveExpansion("tbc")) {
        owner_.visibleItemEntryBaseRef() = tbcVisibleItemBaseFallback();
        owner_.visibleItemStrideRef() = kTbcVisibleItemStride;
    }

    std::array<uint32_t, 19> equipEntries{};
    int nonZero = 0;
    // Prefer authoritative equipped item entry IDs derived from item objects (onlineItems_),
    // because Inventory::ItemDef may not be populated yet if templates haven't been queried.
    for (int i = 0; i < 19; i++) {
        uint64_t itemGuid = owner_.equipSlotGuidsRef()[i];
        if (itemGuid != 0) {
            auto it = owner_.onlineItemsRef().find(itemGuid);
            if (it != owner_.onlineItemsRef().end() && it->second.entry != 0) {
                equipEntries[i] = it->second.entry;
            }
        }
        if (equipEntries[i] == 0) {
            const auto& slot = owner_.inventoryRef().getEquipSlot(static_cast<EquipSlot>(i));
            equipEntries[i] = slot.empty() ? 0u : slot.item.itemId;
        }
        if (equipEntries[i] != 0) nonZero++;
    }
    if (nonZero < 2) return;

    const uint16_t maxKey = owner_.lastPlayerFieldsRef().back().first;
    int bestBase = -1;
    int bestStride = 0;
    int bestMatches = 0;
    int bestMismatches = 9999;
    int bestScore = -999999;

    const std::array<int, 5> strides = isActiveExpansion("tbc")
        ? std::array<int, 5>{16, 2, 3, 4, 1}
        : std::array<int, 5>{2, 3, 4, 1, 16};
    for (int stride : strides) {
        for (const auto& [baseIdxU16, _v] : owner_.lastPlayerFieldsRef()) {
            const int base = static_cast<int>(baseIdxU16);
            if (base + 18 * stride > static_cast<int>(maxKey)) continue;

            int matches = 0;
            int mismatches = 0;
            for (int s = 0; s < 19; s++) {
                uint32_t want = equipEntries[s];
                if (want == 0) continue;
                const uint16_t idx = static_cast<uint16_t>(base + s * stride);
                auto it = owner_.lastPlayerFieldsRef().find(idx);
                if (it == owner_.lastPlayerFieldsRef().end()) continue;
                if (it->second == want) {
                    matches++;
                } else if (it->second != 0) {
                    mismatches++;
                }
            }

            int score = matches * 2 - mismatches * 3;
            if (score > bestScore ||
                (score == bestScore && matches > bestMatches) ||
                (score == bestScore && matches == bestMatches && mismatches < bestMismatches) ||
                (score == bestScore && matches == bestMatches && mismatches == bestMismatches && base < bestBase)) {
                bestScore = score;
                bestMatches = matches;
                bestMismatches = mismatches;
                bestBase = base;
                bestStride = stride;
            }
        }
    }

    if (bestMatches >= 2 && bestBase >= 0 && bestStride > 0 && bestMismatches <= 1) {
        owner_.visibleItemEntryBaseRef() = bestBase;
        owner_.visibleItemStrideRef() = bestStride;
        owner_.visibleItemLayoutVerifiedRef() = true;
        LOG_INFO("Detected PLAYER_VISIBLE_ITEM entry layout: base=", owner_.visibleItemEntryBaseRef(),
                 " stride=", owner_.visibleItemStrideRef(), " (matches=", bestMatches,
                 " mismatches=", bestMismatches, " score=", bestScore, ")");

        // Backfill existing player entities already in view.
        for (const auto& [guid, ent] : owner_.getEntityManager().getEntities()) {
            if (!ent || ent->getType() != ObjectType::PLAYER) continue;
            if (guid == owner_.getPlayerGuid()) continue;
            updateOtherPlayerVisibleItems(guid, ent->getFields());
        }
    }
    // If heuristic didn't find a match, keep using the default WotLK layout (base=284, stride=2).
}

void InventoryHandler::updateOtherPlayerVisibleItems(uint64_t guid, const FlatFieldMap& fields) {
    if (guid == 0 || guid == owner_.getPlayerGuid()) return;

    // Use the current base/stride (defaults are correct for WotLK 3.3.5a: base=284, stride=2).
    // The heuristic may refine these later, but we proceed immediately with whatever values
    // are set rather than waiting for verification.
    const bool valuesAreDisplayIds = usesVisibleItemDisplayIds();
    int base = owner_.visibleItemEntryBaseRef();
    int stride = owner_.visibleItemStrideRef();
    if (isActiveExpansion("tbc") && !owner_.visibleItemLayoutVerifiedRef()) {
        const int kTbcFallbackBase = tbcVisibleItemBaseFallback();
        constexpr int kTbcFallbackStride = kTbcVisibleItemStride;
        if (base != kTbcFallbackBase || stride != kTbcFallbackStride) {
            base = kTbcFallbackBase;
            stride = kTbcFallbackStride;
            owner_.visibleItemEntryBaseRef() = base;
            owner_.visibleItemStrideRef() = stride;
        }
    }
    if (base < 0 || stride <= 0) return; // Defensive: should never happen with defaults.

    std::array<uint32_t, 19> newEntries{};
    for (int s = 0; s < 19; s++) {
        uint16_t idx = static_cast<uint16_t>(base + s * stride);
        auto it = fields.find(idx);
        if (it != fields.end()) newEntries[s] = it->second;
    }

    int nonZero = 0;
    for (uint32_t e : newEntries) { if (e != 0) nonZero++; }
    const bool hasVisibleBodySlot =
        newEntries[2] != 0 ||  // shoulders
        newEntries[4] != 0 ||  // chest
        newEntries[6] != 0 ||  // legs
        newEntries[7] != 0 ||  // feet
        newEntries[9] != 0 ||  // hands
        newEntries[15] != 0 || // main hand
        newEntries[16] != 0;   // off hand
    const bool sparseTbcVisible =
        valuesAreDisplayIds &&
        (nonZero == 0 || nonZero < 4 || (!hasVisibleBodySlot && nonZero < 8));
    const bool hasInspectedEntries =
        owner_.inspectedPlayerItemEntriesRef().find(guid) != owner_.inspectedPlayerItemEntriesRef().end();

    // Dump raw fields around visible item range to find the correct offset
    static int dumpCount = 0;
    if (dumpCount < 3 && fields.size() > 20) {
        dumpCount++;
        std::string dump;
        for (const auto& [idx, val] : fields) {
            if (idx >= 270 && idx <= 340 && val != 0) {
                char buf[32];
                snprintf(buf, sizeof(buf), " [%u]=%u", idx, val);
                dump += buf;
            }
        }
        LOG_DEBUG("RAW FIELDS 270-340:", dump);
    }
    static int diagDumpCount = 0;
    if (visibleEquipmentFieldDiagEnabled() && diagDumpCount < 8 && fields.size() > 20) {
        diagDumpCount++;
        std::ostringstream dump;
        int emitted = 0;
        uint16_t maxField = 0;
        for (const auto& [idx, val] : fields) {
            if (idx > maxField) maxField = idx;
            if (idx < 220 || idx > 760 || val == 0) continue;
            dump << " [" << idx << "]=" << val;
            if (++emitted >= 180) {
                dump << " ...";
                break;
            }
        }
        LOG_INFO("VISIBLE EQUIP FIELD DIAG guid=0x", std::hex, guid, std::dec,
                 " fieldCount=", fields.size(), " maxField=", maxField,
                 " base=", base, " stride=", stride,
                 " verified=", owner_.visibleItemLayoutVerifiedRef(),
                 " nonZero[220-760]=", dump.str());
    }

    if (nonZero > 0) {
        LOG_DEBUG("updateOtherPlayerVisibleItems: guid=0x", std::hex, guid, std::dec,
                 " nonZero=", nonZero, " base=", base, " stride=", stride,
                 " values=", (valuesAreDisplayIds ? "displayIds" : "itemEntries"),
                 " head=", newEntries[0], " shoulders=", newEntries[2],
                 " chest=", newEntries[4], " legs=", newEntries[6],
                 " mainhand=", newEntries[15], " offhand=", newEntries[16]);
    }

    if (sparseTbcVisible) {
        if (!hasInspectedEntries && owner_.getSocket() && owner_.getState() == WorldState::IN_WORLD) {
            owner_.pendingAutoInspectRef().insert(guid);
            LOG_DEBUG("updateOtherPlayerVisibleItems: guid=0x", std::hex, guid, std::dec,
                      " sparse TBC visible fields (nonZero=", nonZero,
                      ", base=", base, ", stride=", stride,
                      ") - queuing auto-inspect");
        } else if (hasInspectedEntries) {
            LOG_DEBUG("updateOtherPlayerVisibleItems: guid=0x", std::hex, guid, std::dec,
                      " sparse TBC visible fields ignored; inspect gear already cached");
        }
    }

    bool changed = false;
    auto& old = owner_.otherPlayerVisibleItemEntriesRef()[guid];
    if (old != newEntries) {
        old = newEntries;
        changed = true;
    }

    if (valuesAreDisplayIds) {
        if (sparseTbcVisible && hasInspectedEntries) {
            return;
        }
        if (changed) {
            owner_.otherPlayerVisibleDirtyRef().insert(guid);
            emitOtherPlayerEquipment(guid);
        }
        return;
    }

    // Request item templates for any new visible entries.
    for (uint32_t entry : newEntries) {
        if (entry == 0) continue;
        if (!owner_.itemInfoCacheRef().count(entry) && !owner_.pendingItemQueriesRef().count(entry)) {
            queryItemInfo(entry, 0);
        }
    }

    // Only fall back to auto-inspect if ALL extracted entries are zero (server didn't
    // send visible item fields at all). If we got at least one non-zero entry, the
    // update-field approach is working and inspect is unnecessary.
    if (nonZero == 0) {
        LOG_DEBUG("updateOtherPlayerVisibleItems: guid=0x", std::hex, guid, std::dec,
                  " all entries zero (base=", base, " stride=", stride,
                  " fieldCount=", fields.size(), ") — queuing auto-inspect");
        if (owner_.getSocket() && owner_.getState() == WorldState::IN_WORLD) {
            owner_.pendingAutoInspectRef().insert(guid);
        }
    }

    if (changed) {
        owner_.otherPlayerVisibleDirtyRef().insert(guid);
        emitOtherPlayerEquipment(guid);
    }
}

void InventoryHandler::cacheInspectedPlayerEquipment(uint64_t guid, const std::array<uint32_t, 19>& itemEntries) {
    if (guid == 0) return;

    owner_.inspectedPlayerItemEntriesRef()[guid] = itemEntries;

    std::array<uint32_t, 19> displayIds{};
    std::array<uint8_t, 19> invTypes{};
    int entries = 0;
    int resolved = 0;

    for (int s = 0; s < 19; ++s) {
        const uint32_t entry = itemEntries[s];
        if (entry == 0) continue;
        entries++;

        auto infoIt = owner_.itemInfoCacheRef().find(entry);
        if (infoIt != owner_.itemInfoCacheRef().end()) {
            displayIds[s] = infoIt->second.displayInfoId;
            invTypes[s] = static_cast<uint8_t>(infoIt->second.inventoryType);
            resolved++;
            continue;
        }

        queryItemInfo(entry, 0);
    }

    LOG_DEBUG("cacheInspectedPlayerEquipment: guid=0x", std::hex, guid, std::dec,
              " entries=", entries, " resolved=", resolved,
              " head=", displayIds[0], " shoulders=", displayIds[2],
              " chest=", displayIds[4], " legs=", displayIds[6],
              " mainhand=", displayIds[15], " offhand=", displayIds[16]);

    if (resolved > 0 && owner_.playerEquipmentCallbackRef()) {
        owner_.playerEquipmentCallbackRef()(guid, displayIds, invTypes);
    }
}

void InventoryHandler::emitOtherPlayerEquipment(uint64_t guid) {
    if (!owner_.playerEquipmentCallbackRef()) return;
    auto it = owner_.otherPlayerVisibleItemEntriesRef().find(guid);
    if (it == owner_.otherPlayerVisibleItemEntriesRef().end()) return;

    std::array<uint32_t, 19> displayIds{};
    std::array<uint8_t, 19> invTypes{};
    if (usesVisibleItemDisplayIds()) {
        displayIds = it->second;
        invTypes = inferredVisibleInventoryTypes();
        int nonZeroDisplay = 0;
        for (uint32_t displayId : displayIds) {
            if (displayId != 0) nonZeroDisplay++;
        }

        LOG_DEBUG("emitOtherPlayerEquipment: guid=0x", std::hex, guid, std::dec,
                 " TBC displayIds=", nonZeroDisplay,
                 " head=", displayIds[0], " shoulders=", displayIds[2],
                 " chest=", displayIds[4], " legs=", displayIds[6],
                 " mainhand=", displayIds[15], " offhand=", displayIds[16]);

        if (nonZeroDisplay == 0) return;
        owner_.playerEquipmentCallbackRef()(guid, displayIds, invTypes);
        owner_.otherPlayerVisibleDirtyRef().erase(guid);
        return;
    }

    bool anyEntry = false;
    int resolved = 0, unresolved = 0;

    for (int s = 0; s < 19; s++) {
        uint32_t entry = it->second[s];
        if (entry == 0) continue;
        anyEntry = true;
        auto infoIt = owner_.itemInfoCacheRef().find(entry);
        if (infoIt == owner_.itemInfoCacheRef().end()) { unresolved++; continue; }
        displayIds[s] = infoIt->second.displayInfoId;
        invTypes[s] = static_cast<uint8_t>(infoIt->second.inventoryType);
        resolved++;
    }

    LOG_DEBUG("emitOtherPlayerEquipment: guid=0x", std::hex, guid, std::dec,
             " entries=", (anyEntry ? "yes" : "none"),
             " resolved=", resolved, " unresolved=", unresolved,
             " head=", displayIds[0], " shoulders=", displayIds[2],
             " chest=", displayIds[4], " legs=", displayIds[6],
             " mainhand=", displayIds[15], " offhand=", displayIds[16]);

    // Don't emit all-zero displayIds — that strips existing equipment for no reason.
    // Wait until at least one item resolves before applying.
    if (anyEntry && resolved == 0) {
        LOG_DEBUG("emitOtherPlayerEquipment: skipping all-zero emit (waiting for item queries)");
        return;
    }

    owner_.playerEquipmentCallbackRef()(guid, displayIds, invTypes);
    owner_.otherPlayerVisibleDirtyRef().erase(guid);

    // If we had entries but couldn't resolve any templates, also try inspect as a fallback.
    if (anyEntry && !resolved) {
        owner_.pendingAutoInspectRef().insert(guid);
    }
}

void InventoryHandler::emitAllOtherPlayerEquipment() {
    if (!owner_.playerEquipmentCallbackRef()) return;
    for (const auto& [guid, _] : owner_.otherPlayerVisibleItemEntriesRef()) {
        emitOtherPlayerEquipment(guid);
    }
}

// ============================================================
// Moved opcode handlers (from GameHandler::registerOpcodeHandlers)
// ============================================================

void InventoryHandler::handleTrainerBuySucceeded(network::Packet& packet) {
    /*uint64_t guid =*/ packet.readUInt64();
    uint32_t spellId = packet.readUInt32();
    if (owner_.getSpellHandler() && !owner_.getSpellHandler()->hasKnownSpell(spellId)) {
        owner_.getSpellHandler()->addKnownSpell(spellId);
    }
    const std::string& name = owner_.getSpellName(spellId);
    if (!name.empty())
        owner_.addSystemChatMessage("You have learned " + name + ".");
    else
        owner_.addSystemChatMessage("Spell learned.");
    if (auto* ac = owner_.services().audioCoordinator)
        if (auto* sfx = ac->getUiSoundManager()) sfx->playQuestActivate();
    owner_.fireAddonEvent("TRAINER_UPDATE", {});
    owner_.fireAddonEvent("SPELLS_CHANGED", {});
}

void InventoryHandler::handleTrainerBuyFailed(network::Packet& packet) {
    /*uint64_t trainerGuid =*/ packet.readUInt64();
    uint32_t spellId = packet.readUInt32();
    uint32_t errorCode = 0;
    if (packet.hasRemaining(4))
        errorCode = packet.readUInt32();
    const std::string& spellName = owner_.getSpellName(spellId);
    std::string msg = "Cannot learn ";
    if (!spellName.empty()) msg += spellName;
    else msg += "spell #" + std::to_string(spellId);
    // SMSG_TRAINER_BUY_FAILED reason codes (WoW 3.3.5a):
    //   0 = trainer service unavailable / cannot learn (requirements unmet, e.g. skill too low)
    //   1 = not enough money
    //   2 = does not meet requirements (class/race/level/skill)
    if (errorCode == 1) msg += " (not enough money)";
    else if (errorCode == 2) msg += " (requirements not met)";
    else msg += " (requirements not met)";
    owner_.addUIError(msg);
    owner_.addSystemChatMessage(msg);
    if (auto* ac = owner_.services().audioCoordinator)
        if (auto* sfx = ac->getUiSoundManager()) sfx->playError();
    if (errorCode == 1)
        owner_.playErrorSpeech(audio::PlayerErrorSpeech::NOT_ENOUGH_MONEY);
    else
        owner_.playErrorSpeech(audio::PlayerErrorSpeech::CANT_LEARN_SPELL);
}

// ============================================================
// Methods moved from GameHandler
// ============================================================

void InventoryHandler::initiateTrade(uint64_t targetGuid) {
    if (!owner_.isInWorld()) {
        LOG_WARNING("Cannot initiate trade: not in world or not connected");
        return;
    }

    if (targetGuid == 0) {
        owner_.addSystemChatMessage("You must target a player to trade with.");
        return;
    }

    auto packet = InitiateTradePacket::build(targetGuid);
    owner_.getSocket()->send(packet);
    owner_.addSystemChatMessage("Requesting trade with target.");
    LOG_INFO("Initiated trade with target: 0x", std::hex, targetGuid, std::dec);
}

uint32_t InventoryHandler::getTempEnchantRemainingMs(uint32_t slot) const {
    uint64_t nowMs = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
    for (const auto& t : owner_.tempEnchantTimersRef()) {
        if (t.slot == slot) {
            return (t.expireMs > nowMs)
                ? static_cast<uint32_t>(t.expireMs - nowMs) : 0u;
        }
    }
    return 0u;
}

void InventoryHandler::addMoneyCopper(uint32_t amount) {
    if (amount == 0) return;
    owner_.playerMoneyCopperRef() += amount;
    uint32_t gold = amount / 10000;
    uint32_t silver = (amount / 100) % 100;
    uint32_t copper = amount % 100;
    std::string msg = "You receive ";
    msg += std::to_string(gold) + "g ";
    msg += std::to_string(silver) + "s ";
    msg += std::to_string(copper) + "c.";
    owner_.addSystemChatMessage(msg);
    owner_.fireAddonEvent("CHAT_MSG_MONEY", {msg});
}

// ============================================================
// Repair cost estimation from DBC data
// ============================================================

void InventoryHandler::loadRepairDbc() const {
    if (repairDbcLoaded_) return;
    repairDbcLoaded_ = true;

    auto* am = owner_.services().assetManager;
    if (!am || !am->isInitialized()) return;

    // DurabilityCosts.dbc: field 0 = itemLevel (key), fields 1-29 = cost multipliers
    // Columns 1-21 = weapon subclass (0-20), columns 22-29 = armor subclass (0-7)
    auto costsDbc = am->loadDBC("DurabilityCosts.dbc");
    if (costsDbc && costsDbc->isLoaded()) {
        uint32_t count = costsDbc->getRecordCount();
        for (uint32_t i = 0; i < count; ++i) {
            uint32_t itemLevel = costsDbc->getUInt32(i, 0);
            std::array<uint32_t, 29> mults{};
            for (uint32_t f = 0; f < 29; ++f)
                mults[f] = costsDbc->getUInt32(i, f + 1);
            durabilityCosts_[itemLevel] = mults;
        }
    }

    // DurabilityQuality.dbc: field 0 = id (key), field 1 = quality_mod (float)
    auto qualDbc = am->loadDBC("DurabilityQuality.dbc");
    if (qualDbc && qualDbc->isLoaded()) {
        uint32_t count = qualDbc->getRecordCount();
        for (uint32_t i = 0; i < count; ++i) {
            uint32_t id = qualDbc->getUInt32(i, 0);
            float mod = qualDbc->getFloat(i, 1);
            durabilityQuality_[id] = mod;
        }
    }
}

uint32_t InventoryHandler::estimateItemRepairCost(uint64_t itemGuid) const {
    auto itemIt = owner_.onlineItemsRef().find(itemGuid);
    if (itemIt == owner_.onlineItemsRef().end()) return 0;
    const auto& item = itemIt->second;

    if (item.maxDurability == 0 || item.curDurability >= item.maxDurability) return 0;
    uint32_t lostDur = item.maxDurability - item.curDurability;

    auto infoIt = owner_.itemInfoCacheRef().find(item.entry);
    if (infoIt == owner_.itemInfoCacheRef().end()) return 0;
    const auto& info = infoIt->second;

    loadRepairDbc();

    // Look up DurabilityCosts multiplier for this itemLevel
    auto costIt = durabilityCosts_.find(info.itemLevel);
    if (costIt == durabilityCosts_.end()) return 0;

    // Determine column index: weapon (class=2) uses subClass directly,
    // armor (class=4) uses subClass + 21
    uint32_t colIndex = 0;
    if (info.itemClass == 2) { // ITEM_CLASS_WEAPON
        colIndex = info.subClass;
    } else if (info.itemClass == 4) { // ITEM_CLASS_ARMOR
        colIndex = info.subClass + 21;
    } else {
        return 0; // only weapons and armor have durability
    }
    if (colIndex >= 29) return 0;

    uint32_t dmultiplier = costIt->second[colIndex];
    if (dmultiplier == 0) return 0;

    // Quality modifier lookup: index is (quality + 1) * 2
    uint32_t qualIndex = (info.quality + 1) * 2;
    auto qualIt = durabilityQuality_.find(qualIndex);
    if (qualIt == durabilityQuality_.end()) return 0;
    float qualMod = qualIt->second;

    uint32_t cost = static_cast<uint32_t>(lostDur * dmultiplier * qualMod);
    if (cost == 0 && lostDur > 0) cost = 1; // minimum 1 copper
    return cost;
}

uint32_t InventoryHandler::estimateRepairAllCost() const {
    uint32_t total = 0;
    for (const auto& [guid, info] : owner_.onlineItemsRef()) {
        total += estimateItemRepairCost(guid);
    }
    return total;
}

} // namespace game
} // namespace wowee
