#pragma once

#include "game/world_packets.hpp"
#include "game/opcode_table.hpp"
#include "game/inventory.hpp"
#include "game/handler_types.hpp"
#include "network/packet.hpp"
#include <array>
#include <chrono>
#include <deque>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace wowee {
namespace game {

class GameHandler;

class InventoryHandler {
public:
    using PacketHandler = std::function<void(network::Packet&)>;
    using DispatchTable = std::unordered_map<LogicalOpcode, PacketHandler>;

    explicit InventoryHandler(GameHandler& owner);

    /// Announce that the bags changed, one event per bag.
    ///
    /// WoW passes the bag that changed as the event's first argument, and the
    /// interface redraws only the bag whose id matches it - so an event with no
    /// id redraws nothing at all. That is why an item dragged to a new slot
    /// stayed drawn in its old one until the bag was closed and reopened.
    void fireBagUpdates();

    void registerOpcodes(DispatchTable& table);

    /// Advances the fallback that announces looted money when the server
    /// never sends SMSG_LOOT_MONEY_NOTIFY.
    ///
    /// Taking gold arms a 0.4s timer here and nothing ticked it: GameHandler
    /// had its own copy of these members and ticked those, and its copy is
    /// never set to anything but zero. So on a server that does not send the
    /// notify, looting money said nothing and played no coin.
    void tickLootMoneyFallback(float deltaTime);

    // ---- Item text (books / readable items) ----
    bool isItemTextOpen() const { return itemTextOpen_; }
    const std::string& getItemText() const { return itemText_; }
    void closeItemText();
    void queryItemText(uint64_t itemGuid);

    // ---- Trade ----
    enum class TradeStatus : uint8_t {
        None = 0, PendingIncoming, Open, Accepted, Complete
    };

    // WoW trade window: 7 slots total. Slots 0-5 are transferred to the partner;
    // slot 6 (TRADE_SLOT_NONTRADED) is the "will not be traded" slot - you place your
    // own item there so the partner can enchant/craft on it without it changing hands.
    static constexpr int TRADE_SLOT_COUNT        = 7;
    static constexpr int TRADE_SLOT_NONTRADED    = 6;

    struct TradeSlot {
        uint32_t itemId      = 0;
        uint32_t displayId   = 0;
        uint32_t stackCount  = 0;
        uint64_t itemGuid    = 0;
    };

    TradeStatus getTradeStatus() const { return tradeStatus_; }
    bool hasPendingTradeRequest() const { return tradeStatus_ == TradeStatus::PendingIncoming; }
    bool isTradeOpen() const { return tradeStatus_ == TradeStatus::Open || tradeStatus_ == TradeStatus::Accepted; }
    const std::string& getTradePeerName() const { return tradePeerName_; }
    uint64_t getTradePeerGuid() const { return tradePeerGuid_; }
    const std::array<TradeSlot, TRADE_SLOT_COUNT>& getMyTradeSlots() const { return myTradeSlots_; }
    const std::array<TradeSlot, TRADE_SLOT_COUNT>& getPeerTradeSlots() const { return peerTradeSlots_; }
    uint64_t getMyTradeGold() const { return myTradeGold_; }
    uint64_t getPeerTradeGold() const { return peerTradeGold_; }
    void acceptTradeRequest();
    void declineTradeRequest();
    void acceptTrade();
    void unacceptTrade();
    void cancelTrade();
    void setTradeItem(uint8_t tradeSlot, uint8_t srcBag, uint8_t srcSlot);
    void clearTradeItem(uint8_t tradeSlot);
    void setTradeGold(uint64_t amount);

    // ---- Loot ----
    void lootTarget(uint64_t targetGuid);
    /// Take the coin on the corpse. Its own request, with no slot:
    /// money is not one of the numbered loot slots on the wire even
    /// though the interface shows it as one.
    void lootMoney();
    /// Drop a temporary weapon enchant - a sharpening stone, poison, or a
    /// shaman's weapon imbue. Slot zero is the main hand and one the off hand,
    /// which is how the request numbers them; the interface counts from one.
    void cancelTempEnchantment(uint8_t handIndex);
    void closeLoot();
    bool isLootWindowOpen() const { return lootWindowOpen_; }
    const LootResponseData& getCurrentLoot() const { return currentLoot_; }
    void setAutoLoot(bool enabled) { autoLoot_ = enabled; }
    bool isAutoLoot() const { return autoLoot_; }
    void setAutoSellGrey(bool enabled) { autoSellGrey_ = enabled; }
    bool isAutoSellGrey() const { return autoSellGrey_; }
    void setAutoRepair(bool enabled) { autoRepair_ = enabled; }
    bool isAutoRepair() const { return autoRepair_; }

    // Master loot candidates (from SMSG_LOOT_MASTER_LIST)
    const std::vector<uint64_t>& getMasterLootCandidates() const { return masterLootCandidates_; }
    bool hasMasterLootCandidates() const { return !masterLootCandidates_.empty(); }
    void lootMasterGive(uint8_t lootSlot, uint64_t targetGuid);

    // Group loot roll (aliased from handler_types.hpp)
    using LootRollEntry = game::LootRollEntry;
    /// The roll the interface means, or null when it has already closed.
    ///
    /// Every roll the player has open is kept. This used to hold one: a dungeon
    /// boss puts up several at once, each new roll overwrote the last, and a
    /// window already on screen then described the item from the roll after it
    /// - and rolling Need on it rolled on that one instead.
    const LootRollEntry* getLootRoll(int rollId) const {
        for (const auto& roll : lootRolls_) {
            if (roll.rollId == rollId) return &roll;
        }
        return nullptr;
    }
    void sendLootRoll(uint64_t objectGuid, uint32_t slot, uint8_t rollType);

    // ---- Equipment Sets (aliased from handler_types.hpp) ----
    using EquipmentSetInfo = game::EquipmentSetInfo;
    const std::vector<EquipmentSetInfo>& getEquipmentSets() const { return equipmentSetInfo_; }
    /// The items a saved set holds, by equipment slot, and which slots it was
    /// told to leave alone. Held as item guids, which is what the server sends.
    /// Null when no set has that id.
    const std::array<uint64_t, 19>* getEquipmentSetItems(uint32_t setId) const {
        for (const auto& set : equipmentSets_) {
            if (set.setId == setId) return &set.itemGuids;
        }
        return nullptr;
    }
    uint32_t getEquipmentSetIgnoreMask(uint32_t setId) const {
        for (const auto& set : equipmentSets_) {
            if (set.setId == setId) return set.ignoreSlotMask;
        }
        return 0;
    }
    bool supportsEquipmentSets() const;
    void useEquipmentSet(uint32_t setId);
    void saveEquipmentSet(const std::string& name, const std::string& iconName = "INV_Misc_QuestionMark",
                          uint64_t existingGuid = 0, uint32_t setIndex = 0xFFFFFFFF);
    void deleteEquipmentSet(uint64_t setGuid);

    // ---- Vendor ----
    struct BuybackItem {
        uint64_t itemGuid = 0;
        ItemDef item;
        uint32_t count = 1;
        // Inventory slot sent in CMSG_BUYBACK_ITEM (74..85).  This is server
        // identity, not the item's current row in the buyback UI.
        uint32_t wireSlot = 0;
    };
    void openVendor(uint64_t npcGuid);
    void closeVendor();
    // Drop the session-persistent buyback mirror (fresh character world entry).
    void clearBuybackState();
    void buyItem(uint64_t vendorGuid, uint32_t itemId, uint32_t slot, uint32_t count);
    void sellItem(uint64_t vendorGuid, uint64_t itemGuid, uint32_t count);
    void sellItemBySlot(int backpackIndex);
    void sellItemInBag(int bagIndex, int slotIndex);
    void buyBackItem(uint32_t buybackSlot);
    /// Tell the interface durability moved - the armour indicator redraws
    /// from an event, not by polling.
    void announceDurabilityChange();
    void repairItem(uint64_t vendorGuid, uint64_t itemGuid);
    void repairAll(uint64_t vendorGuid, bool useGuildBank = false);
    uint32_t estimateRepairAllCost() const;
    const std::deque<BuybackItem>& getBuybackItems() const { return buybackItems_; }
    // ---- Equipping, and the prompt before something binds ----
    //
    // `confirmed` is how a caller says the player has already been asked. Both
    // interfaces ask - this client's own popup and FrameXML's EQUIP_BIND - and
    // both come back through here, so without it the answer to the prompt
    // raises the prompt again.
    void equipItemToSlot(uint64_t itemGuid, uint8_t equipSlot);
    void autoEquipItemBySlot(int backpackIndex, bool confirmed = false);
    void autoEquipItemInBag(int bagIndex, int slotIndex, bool confirmed = false);

    /// Whether equipping what is in a slot would bind it - ItemDef carries
    /// the rule; these two just find the item.
    bool equipWouldBindFromBackpack(int backpackIndex) const;
    bool equipWouldBindFromBag(int bagIndex, int slotIndex) const;

    /// The equip held back waiting for an answer. One at a time - FrameXML's
    /// dialog is exclusive and this client's is modal, so a second cannot be
    /// raised while the first is up.
    struct PendingEquip {
        bool    active  = false;
        bool    fromBag = false;
        int     bag     = 0;
        int     slot    = 0;
        uint8_t wireSlot = 0;   ///< what the event carried, for the reply
    };
    // ---- Looting, and the prompt before something binds ----
    //
    // Same shape as the equip prompt above and for the same reason: neither
    // interface warned before taking a bind-on-pickup item.
    void lootItem(uint8_t slotIndex, bool confirmed = false);
    /// Send the loot request that was held back. The dialog passes the slot it
    /// was shown for, but only one can be waiting, so the held one is enough -
    /// and going back through lootItem with the slot would raise the prompt
    /// again, which is a loop rather than a confirmation.
    void confirmPendingLoot();

    /// Send the use that was held back by the bind-on-use prompt. USE_BIND
    /// carries no slot at all - FrameXML shows it and calls this with nothing -
    /// so the held request is the only record of what was being used.
    void confirmBindOnUse();

    /// Apply the enchant that was held back by the replace prompt.
    void replaceEnchant();

    void equipPendingItem();
    void cancelPendingEquip();
    /// `unitTarget` names who the item is used on, and overrides the default
    /// the item's own class implies. Only the interface's /use handling passes
    /// it - `/use [target=Bob] Heavy Runecloth Bandage` - and zero keeps the
    /// behaviour every other caller has always had.
    void useItemBySlot(int backpackIndex, bool confirmed = false,
                       uint64_t unitTarget = 0);
    /// Use a key from the keyring. Its own entry point because the keyring is
    /// addressed by a wire slot of its own past the bags, and useItemBySlot
    /// bounds-checks against the backpack and would silently do nothing.
    void useKeyringItem(int index, bool confirmed = false, uint64_t unitTarget = 0);
    /// Use something already worn - a trinket, or anything else with an on-use.
    /// Addressed by its equipment slot, which is its own wire slot below the
    /// backpack's.
    void useEquippedItem(int equipSlot, bool confirmed = false, uint64_t unitTarget = 0);
    void useItemInBag(int bagIndex, int slotIndex, bool confirmed = false,
                      uint64_t unitTarget = 0);

    // ---- Item-targeted item use (sharpening stones, weightstones, weapon oils) ----
    /// True while a used item is waiting for the player to pick the item it applies to.
    bool isAwaitingItemTarget() const;
    /// Entry of the item awaiting a target (0 if none) - drives the targeting cursor.
    uint32_t getPendingItemTargetSourceItemId() const;
    void cancelItemTargeting();

    // ---- Unit-targeted item use (bandages, treats, quest items) ----
    /// True while a used item is waiting for the player to click a unit.
    ///
    /// An item whose spell needs a unit and no unit selected is not a refusal
    /// in WoW: the cursor changes and the next click on someone chooses them.
    /// Without this the item simply did nothing.
    bool isAwaitingUnitTarget() const;
    /// Entry of the item awaiting a unit (0 if none) - drives the cursor.
    uint32_t getPendingUnitTargetSourceItemId() const;
    void cancelUnitTargeting();
    /// Sends the parked CMSG_USE_ITEM against the unit the player clicked.
    void completeItemUseOnUnit(uint64_t targetUnitGuid);

    /// Arm item targeting for a spell that must be cast at an item. The cast is
    /// sent once the player picks one.
    void beginSpellItemTargeting(uint32_t spellId, const std::string& spellName);
    /// Sends the parked CMSG_USE_ITEM with TARGET_FLAG_ITEM against targetItemGuid.
    void completeItemUseOnItem(uint64_t targetItemGuid, bool confirmed = false);

    void openItemBySlot(int backpackIndex);
    void openItemInBag(int bagIndex, int slotIndex);
    void readItemBySlot(int backpackIndex);
    void readItemInBag(int bagIndex, int slotIndex);
    void destroyItem(uint8_t bag, uint8_t slot, uint8_t count = 1);
    void splitItem(uint8_t srcBag, uint8_t srcSlot, uint8_t count);
    /// Split `count` off a stack into a named slot.
    ///
    /// The interface's SplitContainerItem does not choose a destination: it puts
    /// the split portion on the cursor and the drop decides. Choosing one here
    /// is right only for this client's own bag window, which has no cursor to
    /// put it on.
    void splitItemTo(uint8_t srcBag, uint8_t srcSlot,
                     uint8_t dstBag, uint8_t dstSlot, uint8_t count);
    /// applyLocally=false sends the move without touching the model, for a
    /// caller that has already put the items where this would move them -
    /// see GameHandler::sortBags.
    void swapContainerItems(uint8_t srcBag, uint8_t srcSlot, uint8_t dstBag,
                            uint8_t dstSlot, bool applyLocally = true);
    /// Read and write a model slot by the wire's flat numbering.
    bool readWireSlot(uint8_t container, uint8_t slot, game::ItemDef& out) const;
    bool writeWireSlot(uint8_t container, uint8_t slot, const game::ItemDef& item);
    void swapBagSlots(int srcBagIndex, int dstBagIndex);
    void unequipToBackpack(EquipSlot equipSlot);
    void useItemById(uint32_t itemId, uint64_t unitTarget = 0);
    bool isVendorWindowOpen() const { return vendorWindowOpen_; }
    const ListInventoryData& getVendorItems() const { return currentVendorItems_; }
    void setVendorCanRepair(bool v) { currentVendorItems_.canRepair = v; }
    uint64_t getVendorGuid() const { return currentVendorItems_.vendorGuid; }

    // ---- Mail ----
    // Slots the UI can hold. What can actually be sent depends on the wire
    // format - see maxSendableMailAttachments().
    static constexpr int MAIL_MAX_ATTACHMENTS = 12;

    /// How many attachments this expansion's CMSG_SEND_MAIL can carry. Vanilla
    /// writes a single item GUID, so anything past the first was silently left
    /// in the player's bags while the UI happily accepted twelve.
    static int maxSendableMailAttachments();
    struct MailAttachSlot {
        uint64_t itemGuid = 0;
        game::ItemDef item;
        uint8_t srcBag = 0xFF;
        uint8_t srcSlot = 0;
        [[nodiscard]] bool occupied() const { return itemGuid != 0; }
    };
    bool isMailboxOpen() const { return mailboxOpen_; }
    const std::vector<MailMessage>& getMailInbox() const { return mailInbox_; }
    /// Writable, for the backfill that fills in a mail's sender once the name
    /// packet for that guid arrives.
    std::vector<MailMessage>& mailInboxRef() { return mailInbox_; }
    int getSelectedMailIndex() const { return selectedMailIndex_; }
    void setSelectedMailIndex(int idx) { selectedMailIndex_ = idx; }
    bool isMailComposeOpen() const { return showMailCompose_; }

    /// Whether the compose frame is on screen, without touching the draft.
    ///
    /// FrameXML owns the mail window and says so through SetSendMailShowing as
    /// its two tabs swap. openMailCompose empties the attachments, which is
    /// right when this client's own window opens a fresh letter and wrong on a
    /// tab switch - flipping to the inbox and back would drop what was on it.
    void setMailComposeShowing(bool showing) { showMailCompose_ = showing; }
    void openMailCompose() { showMailCompose_ = true; clearMailAttachments(); }
    void closeMailCompose() { showMailCompose_ = false; clearMailAttachments(); }
    bool hasNewMail() const { return hasNewMail_; }
    void openMailbox(uint64_t guid);
    void closeMailbox();
    /// What the server's mail refusal means, in words.
    ///
    /// SMSG_SEND_MAIL_RESULT carries a number from MailResponseResult and the
    /// client printed it bare - "Failed to send mail (error 19)" says nothing
    /// about what to do differently, and 19 is the one a player can actually
    /// act on: something on the letter cannot be mailed.
    static const char* mailResultText(uint32_t error);

    /// Put paper in the compose frame, so its Send button can be enabled.
    void selectDefaultStationery();

    /// Say why a letter is not going out, and tell the compose frame so it
    /// puts its Send button back.
    void refuseSend(const std::string& reason, const char* logLine);

    void sendMail(const std::string& recipient, const std::string& subject,
                  const std::string& body, uint64_t money, uint64_t cod = 0);
    bool attachItemFromBackpack(int backpackIndex);
    /// Raise MAIL_LOCK_SEND_ITEMS when the attachment still has a refund
    /// window, which posting it ends.
    void noteMailAttachRefundable(int attachIndex);
    bool attachItemFromBag(int bagIndex, int slotIndex);
    bool detachMailAttachment(int attachIndex);
    void clearMailAttachments();

    /// Tell the compose frame its attachments changed.
    ///
    /// The letter being written lives entirely on this side; the send frame
    /// draws its twelve slots, its postage and its Send button from
    /// GetSendMailItem and redraws them only on MAIL_SEND_INFO_UPDATE. Attaching
    /// and detaching used to change the list without saying so, and only the
    /// send - which clears it - announced itself. So an item clicked onto the
    /// letter went onto it and the slot stayed empty, with the item gone from
    /// the cursor as well: it read as the attach having been refused.
    void notifyMailComposeChanged();
    const std::array<MailAttachSlot, 12>& getMailAttachments() const { return mailAttachments_; }
    int getMailAttachmentCount() const;
    void mailTakeMoney(uint32_t mailId);
    void mailTakeItem(uint32_t mailId, uint32_t itemGuidLow);
    void mailDelete(uint32_t mailId);
    void mailReturnToSender(uint32_t mailId);
    void mailMarkAsRead(uint32_t mailId);
    void refreshMailList();

    // ---- Item socketing ----
    //
    // A gem is not put into an item one at a time: the player fills the sockets
    // on screen and then commits all three at once, and until they press the
    // button nothing has happened on the server. So the pending gems are this
    // client's own state, exactly like the mail attachment slots above.
    struct SocketSession {
        bool     open     = false;
        uint64_t itemGuid = 0;   ///< the item whose sockets are on screen
        uint32_t itemId   = 0;   ///< its template, for colours and the portrait
        /// Gems placed but not yet committed, per socket. Guid is what the
        /// server is told; the item id is what the panel draws.
        std::array<uint64_t, 3> newGemGuid{};
        std::array<uint32_t, 3> newGemItemId{};
    };
    const SocketSession& getSocketSession() const { return socketSession_; }
    /// Opens the panel on an item the player owns, wherever it is.
    void openSocketing(uint64_t itemGuid);
    void closeSocketing();
    /// Puts a gem in a socket, or takes back what is in one when guid is zero.
    /// Refuses a gem already sitting in another socket - the server drops the
    /// whole request when two sockets name the same guid.
    bool setSocketGem(int index, uint64_t gemGuid, uint32_t gemItemId);
    /// Commits every pending gem. Nothing is sent when none are pending.
    void acceptSockets();

    // ---- Bank ----
    void openBank(uint64_t guid);
    void closeBank();
    void buyBankSlot();
    // Cost in copper to purchase the bank bag slot at the given 0-based index
    // (BankBagSlotPrices.dbc; identical values across Classic/TBC/WotLK). 0 = free/invalid.
    static uint32_t getBankBagSlotPrice(int slotIndex);
    void depositItem(uint8_t srcBag, uint8_t srcSlot);
    void withdrawItem(uint8_t srcBag, uint8_t srcSlot);
    bool isBankOpen() const { return bankOpen_; }
    uint64_t getBankerGuid() const { return bankerGuid_; }
    int getEffectiveBankSlots() const { return effectiveBankSlots_; }
    int getEffectiveBankBagSlots() const { return effectiveBankBagSlots_; }

    // ---- Guild Bank ----
    void openGuildBank(uint64_t guid);
    void closeGuildBank();
    void queryGuildBankTab(uint8_t tabId);
    void buyGuildBankTab();
    void depositGuildBankMoney(uint32_t amount);
    void withdrawGuildBankMoney(uint32_t amount);
    void guildBankWithdrawItem(uint8_t tabId, uint8_t bankSlot, uint8_t destBag,
                               uint8_t destSlot, uint32_t splitCount = 0);
    /// Put a glyph from a bag slot into a socket.
    ///
    /// There is no separate socketing opcode in 3.3.5: the glyph item is used
    /// with the socket written into the glyphIndex field CMSG_USE_ITEM already
    /// carries, and the server applies the item's spell to that socket. The
    /// wire's bag and slot are the same ones a plain use writes.
    void placeGlyphFromBag(uint8_t wireBag, uint8_t wireSlot, uint32_t socketIndex);

    /// Ask for a tab's info text, and read back what arrived.
    ///
    /// MSG_QUERY_GUILD_BANK_TEXT is a request the server answers with the same
    /// opcode: a tab id and the text. This client could already *write* the
    /// text - SetGuildBankText has sent CMSG_SET_GUILD_BANK_TEXT all along -
    /// and had no way to read it, so a tab's description could be saved and
    /// never seen again.
    void queryGuildBankText(uint8_t tabId);
    const std::string& getGuildBankTabText(uint8_t tabId) const;

    /// Rename a guild bank tab and pick its icon.
    ///
    /// CMSG_GUILD_BANK_UPDATE_TAB: the banker's guid, the tab counted from
    /// zero, then the name and the icon path. AzerothCore drops it unless both
    /// strings are non-empty and the player is standing at the bank, which is
    /// why the empty name the popup allows is refused here rather than sent.
    void setGuildBankTabInfo(uint8_t tabId, const std::string& name,
                             const std::string& icon);
    void guildBankDepositItem(uint8_t tabId, uint8_t bankSlot, uint8_t srcBag, uint8_t srcSlot);
    // Deposit an inventory item into the first free slot of the viewed tab.
    void guildBankDepositFromInventory(uint8_t srcBag, uint8_t srcSlot);
    bool isGuildBankOpen() const { return guildBankOpen_; }
    const GuildBankData& getGuildBankData() const { return guildBankData_; }
    uint8_t getGuildBankActiveTab() const { return guildBankActiveTab_; }
    void setGuildBankActiveTab(uint8_t tab) { guildBankActiveTab_ = tab; }

    // ---- Auction House ----
    void openAuctionHouse(uint64_t guid);
    void closeAuctionHouse();
    void auctionSearch(const std::string& name, uint8_t levelMin, uint8_t levelMax,
                       uint32_t quality, uint32_t itemClass, uint32_t itemSubClass,
                       uint32_t invTypeMask, uint8_t usableOnly, uint32_t offset = 0,
                       const std::vector<AuctionSortKey>& sort = {});
    void auctionSellItem(int backpackIndex, uint32_t bid,
                         uint32_t buyout, uint32_t duration);
    // Post an auction for an item identified by its server GUID, so items in any
    // container (backpack or equipped bags) can be listed, not just the backpack.
    void auctionSellItemByGuid(uint64_t itemGuid, uint32_t stackCount, uint32_t bid,
                               uint32_t buyout, uint32_t duration);
    void auctionPlaceBid(uint32_t auctionId, uint32_t amount);
    void auctionBuyout(uint32_t auctionId, uint32_t buyoutPrice);
    void auctionCancelItem(uint32_t auctionId);
    void auctionListOwnerItems(uint32_t offset = 0);
    void auctionListBidderItems(uint32_t offset = 0);
    bool isAuctionHouseOpen() const { return auctionOpen_; }
    uint64_t getAuctioneerGuid() const { return auctioneerGuid_; }
    const AuctionListResult& getAuctionBrowseResults() const { return auctionBrowseResults_; }
    const AuctionListResult& getAuctionOwnerResults() const { return auctionOwnerResults_; }
    const AuctionListResult& getAuctionBidderResults() const { return auctionBidderResults_; }
    /// Writable, for the one thing that reorders a result set in place: the
    /// panel's own column sort. The server sends a list and the client sorts
    /// it, which is what the real client does too, there is no re-query.
    AuctionListResult& auctionBrowseResultsRef() { return auctionBrowseResults_; }
    AuctionListResult& auctionOwnerResultsRef()  { return auctionOwnerResults_; }
    AuctionListResult& auctionBidderResultsRef() { return auctionBidderResults_; }
    int getAuctionActiveTab() const { return auctionActiveTab_; }
    void setAuctionActiveTab(int tab) { auctionActiveTab_ = tab; }
    float getAuctionSearchDelay() const { return auctionSearchDelayTimer_; }
    // Ticked from GameHandler::update - this member is the authoritative
    // timer (GameHandler used to decrement its own never-set copy, leaving
    // the search button disabled forever after the first search).
    void tickAuctionSearchDelay(float deltaTime) {
        if (auctionSearchDelayTimer_ > 0.0f) {
            auctionSearchDelayTimer_ -= deltaTime;
            if (auctionSearchDelayTimer_ < 0.0f) auctionSearchDelayTimer_ = 0.0f;
        }
    }

    // ---- Trainer ----
    struct TrainerTab {
        std::string name;
        std::vector<const TrainerSpell*> spells;
    };
    bool isTrainerWindowOpen() const { return trainerWindowOpen_; }
    const TrainerListData& getTrainerSpells() const { return currentTrainerList_; }
    void trainSpell(uint32_t spellId);
    void closeTrainer();
    const std::vector<TrainerTab>& getTrainerTabs() const { return trainerTabs_; }
    void resetTradeState();

    // ---- Methods moved from GameHandler ----
    void initiateTrade(uint64_t targetGuid);
    uint32_t getTempEnchantRemainingMs(uint32_t slot) const;
    void addMoneyCopper(uint32_t amount);

    // ---- Inventory field / rebuild methods (moved from GameHandler) ----
    void queryItemInfo(uint32_t entry, uint64_t guid);
    uint64_t resolveOnlineItemGuid(uint32_t itemId) const;
    void detectInventorySlotBases(const FlatFieldMap& fields);
    bool applyInventoryFields(const FlatFieldMap& fields);
    void extractContainerFields(uint64_t containerGuid, const FlatFieldMap& fields);
    ItemDef buildItemDef(uint32_t entry, uint32_t stackCount, uint32_t curDur, uint32_t maxDur, uint64_t guid,
                         uint32_t flags = 0, int32_t randomPropertyId = 0, uint32_t suffixFactor = 0);
    void rebuildOnlineInventory();
    /// Announce the bank slots that moved, once their items are current.
    void fireBankSlotEvents();
    void maybeDetectVisibleItemLayout();
    void updateOtherPlayerVisibleItems(uint64_t guid, const FlatFieldMap& fields);
    void cacheInspectedPlayerEquipment(uint64_t guid, const std::array<uint32_t, 19>& itemEntries);
    void emitOtherPlayerEquipment(uint64_t guid);
    /// The same resolution, answered rather than announced - see the
    /// definition for why both shapes exist.
    ///
    /// The player's own guid is answered too, from their own equipped slots -
    /// see resolvePlayerEquipment.
    bool resolveOtherPlayerEquipment(uint64_t guid,
                                     std::array<uint32_t, 19>& displayIds,
                                     std::array<uint8_t, 19>& invTypes) const;
    /// What this character is wearing, in the same nineteen slots and the same
    /// order the visible-item fields use.
    ///
    /// Out of the inventory rather than out of that table, because the table
    /// is built from other players' update fields and deliberately holds
    /// nothing for the player - this client knows its own equipment exactly.
    bool resolvePlayerEquipment(std::array<uint32_t, 19>& displayIds,
                                std::array<uint8_t, 19>& invTypes) const;
    void emitAllOtherPlayerEquipment();
    void handleItemQueryResponse(network::Packet& packet);

private:
    // --- Packet handlers ---
    void handleLootResponse(network::Packet& packet);
    void handleLootReleaseResponse(network::Packet& packet);
    void handleLootRemoved(network::Packet& packet);
    void handleListInventory(network::Packet& packet);
    void handleTrainerList(network::Packet& packet);
    void handleItemTextQueryResponse(network::Packet& packet);
    void handleTradeStatus(network::Packet& packet);
    void handleTradeStatusExtended(network::Packet& packet);
    void handleLootRoll(network::Packet& packet);
    void handleLootRollWon(network::Packet& packet);

    /// Tell the interface a roll is over, by the id it was opened with.
    ///
    /// The roll window hides itself on this and nothing else, so without it a
    /// resolved roll stays on screen over a bar that has run down, offering
    /// buttons the server has already stopped listening for.
    LootRollEntry& beginLootRoll(uint64_t objectGuid, uint32_t slot);
    LootRollEntry* findLootRoll(uint64_t objectGuid, uint32_t slot);
    void endLootRoll(uint64_t objectGuid, uint32_t slot);
    void handleShowBank(network::Packet& packet);
    void handleBuyBankSlotResult(network::Packet& packet);
    void handleGuildBankList(network::Packet& packet);
    void handleAuctionHello(network::Packet& packet);
    void handleAuctionListResult(network::Packet& packet);
    void handleAuctionOwnerListResult(network::Packet& packet);
    void handleAuctionBidderListResult(network::Packet& packet);
    void handleAuctionCommandResult(network::Packet& packet);
    void handleShowMailbox(network::Packet& packet);
    void handleMailListResult(network::Packet& packet);
    void handleSendMailResult(network::Packet& packet);
    void handleReceivedMail(network::Packet& packet);
    void handleQueryNextMailTime(network::Packet& packet);
    // Update the unread-mail flag, announcing (chat + sound) once on the rising edge.
    void setHasNewMail(bool value);
    void handleEquipmentSetList(network::Packet& packet);

    void categorizeTrainerSpells();
    void handleTrainerBuySucceeded(network::Packet& packet);
    void handleTrainerBuyFailed(network::Packet& packet);

    // Resolves the item's on-use spell, then either parks the use for item
    // targeting or sends it immediately.
    void dispatchUseItem(uint8_t wowBag, uint8_t wowSlot, uint64_t itemGuid,
                         const ItemDef& item, bool confirmed = false,
                         uint64_t unitTarget = 0);
    void sendUseItem(uint8_t wowBag, uint8_t wowSlot, uint64_t itemGuid, uint32_t spellId,
                     uint64_t targetGuid, uint64_t itemTargetGuid);

    GameHandler& owner_;

    // ---- Item-targeted item use ----
    struct PendingItemTarget {
        uint8_t  bag = 0xFF;
        uint8_t  slot = 0;
        uint64_t itemGuid = 0;
        uint32_t spellId = 0;
        uint32_t itemId = 0;
        std::string itemName;
        /// True when a spell is waiting for its item, rather than an item being
        /// applied to another item. Disenchant, Prospecting, Milling and the
        /// enchant formulas all take an item and are cast, not used.
        bool fromSpell = false;
    };
    // mutable: isAwaitingItemTarget() drops the pending use when out of world.
    mutable std::optional<PendingItemTarget> pendingItemTarget_;
    /// The same, for an item waiting on a unit rather than on another item.
    mutable std::optional<PendingItemTarget> pendingUnitTarget_;

    // Per-equip-slot (permanentEnchant << 32 | temporaryEnchant), so an enchant
    // change marks equipment dirty even though the displayInfoId is unchanged.
    std::array<uint64_t, 19> lastEquipEnchantIds_{};

    // ---- Item text state ----
    bool        itemTextOpen_   = false;
    std::string itemText_;

    // ---- Trade state ----
    TradeStatus tradeStatus_  = TradeStatus::None;
    /// Which side has pressed accept, kept apart because TRADE_ACCEPT_UPDATE
    /// carries both and tradeStatus_ can only say one thing at a time. Ours is
    /// set when the accept goes out - the server echoes an acceptance to the
    /// other player, not back to the one who made it, so there is nothing else
    /// to learn it from.
    bool tradeSelfAccepted_ = false;
    bool tradePartnerAccepted_ = false;
    void fireTradeAcceptUpdate();
    uint64_t    tradePeerGuid_= 0;
    std::string tradePeerName_;
    std::array<TradeSlot, TRADE_SLOT_COUNT> myTradeSlots_{};
    std::array<TradeSlot, TRADE_SLOT_COUNT> peerTradeSlots_{};
    uint64_t myTradeGold_   = 0;
    uint64_t peerTradeGold_ = 0;

    // ---- Loot state ----
    bool lootWindowOpen_ = false;
    bool autoLoot_ = false;
    bool autoSellGrey_ = false;
    bool autoRepair_ = false;
    LootResponseData currentLoot_;
    std::vector<uint64_t> masterLootCandidates_;

    // Group loot roll state
    std::vector<LootRollEntry> lootRolls_;
    /// Never reused within a session, so a window that outlives its roll finds
    /// nothing rather than somebody else's item.
    int           nextLootRollId_ = 1;
    struct LocalLootState {
        LootResponseData data;
        bool moneyTaken = false;
        bool itemAutoLootSent = false;
    };
    std::unordered_map<uint64_t, LocalLootState> localLootState_;
    void announceLootMoney(uint64_t lootGuid, uint32_t amount);
    /// The coin is gone from the open loot window, because the server has said
    /// so. Idempotent: both packets that report it call this.
    void clearLootMoney();
    uint64_t pendingLootMoneyGuid_ = 0;
    uint32_t pendingLootMoneyAmount_ = 0;
    float pendingLootMoneyNotifyTimer_ = 0.0f;
    std::unordered_map<uint64_t, float> recentLootMoneyAnnounceCooldowns_;

    // ---- Vendor state ----
    bool vendorWindowOpen_ = false;
    ListInventoryData currentVendorItems_;
    std::deque<BuybackItem> buybackItems_;
    std::unordered_map<uint64_t, BuybackItem> pendingSellToBuyback_;
    std::array<uint64_t, 12> buybackSlotGuids_{};
    int pendingBuybackSlot_ = -1;
    uint32_t pendingBuybackWireSlot_ = 0;
    uint32_t pendingBuyItemId_ = 0;
    uint32_t pendingBuyItemSlot_ = 0;

    void reconcileBuybackSlots();

    // ---- Mail state ----
    SocketSession socketSession_;
    PendingEquip pendingEquip_;
    /// The use held back waiting for an answer. Kept whole rather than as a
    /// slot pair: the item may be gone from that slot by the time the player
    /// answers, and re-reading it would use whatever moved into its place.
    struct PendingUse {
        bool     active = false;
        uint8_t  wowBag = 0;
        uint8_t  wowSlot = 0;
        uint64_t itemGuid = 0;
        ItemDef  item;
    };
    PendingUse pendingUse_;
    /// The enchant held back waiting for an answer, with the request it was
    /// made from and the item it was aimed at.
    ///
    /// The whole request is moved here rather than left parked in
    /// pendingItemTarget_. FrameXML's REPLACE_ENCHANT has a No with nothing
    /// behind it - no OnCancel, no OnHide - so a refusal is silence, and a
    /// request left waiting for a target it will never be given would be
    /// applied to whatever the player clicked next.
    struct PendingEnchant {
        bool              active = false;
        uint64_t          targetItemGuid = 0;
        PendingItemTarget request;
    };
    PendingEnchant pendingEnchant_;
    bool pendingLootActive_ = false;
    uint8_t pendingLootSlot_ = 0;
    bool mailboxOpen_ = false;
    uint64_t mailboxGuid_ = 0;
    std::vector<MailMessage> mailInbox_;
    int selectedMailIndex_ = -1;
    bool showMailCompose_ = false;
    bool hasNewMail_ = false;
    std::array<MailAttachSlot, MAIL_MAX_ATTACHMENTS> mailAttachments_{};

    // ---- Bank state ----
    bool bankOpen_ = false;
    uint64_t bankerGuid_ = 0;
    std::array<uint64_t, 28> bankSlotGuids_{};
    std::array<uint64_t, 7> bankBagSlotGuids_{};
    /// Bank slots whose guid changed, held until rebuildOnlineInventory has
    /// written the items the bank frame reads. Announcing earlier redraws the
    /// slot from what it held before the move.
    std::vector<int> pendingBankSlotEvents_;
    int effectiveBankSlots_ = 28;
    int effectiveBankBagSlots_ = 7;

    // ---- Guild Bank state ----
    bool guildBankOpen_ = false;
    uint64_t guildBankerGuid_ = 0;
    std::array<std::string, 6> guildBankTabText_{};
    GuildBankData guildBankData_;
    uint8_t guildBankActiveTab_ = 0;

    // ---- Auction House state ----
    bool auctionOpen_ = false;
    uint64_t auctioneerGuid_ = 0;
    AuctionListResult auctionBrowseResults_;
    AuctionListResult auctionOwnerResults_;
    AuctionListResult auctionBidderResults_;
    int auctionActiveTab_ = 0;
    float auctionSearchDelayTimer_ = 0.0f;
    struct AuctionSearchParams {
        std::string name;
        uint8_t levelMin = 0, levelMax = 0;
        uint32_t quality = 0xFFFFFFFF;
        uint32_t itemClass = 0xFFFFFFFF;
        uint32_t itemSubClass = 0xFFFFFFFF;
        uint32_t invTypeMask = 0;
        uint8_t usableOnly = 0;
        uint32_t offset = 0;
        /// Carried so the re-query after a successful bid asks for the same
        /// ordering. Dropping it there would reorder the list under the player
        /// at the moment they bought something.
        std::vector<AuctionSortKey> sort;
    };
    AuctionSearchParams lastAuctionSearch_;
    bool hasAuctionSearch_ = false;  // true after any search (including empty-name browse-all)
    enum class AuctionResultTarget { BROWSE, OWNER, BIDDER };
    AuctionResultTarget pendingAuctionTarget_ = AuctionResultTarget::BROWSE;

    // ---- Trainer state ----
    bool trainerWindowOpen_ = false;
    TrainerListData currentTrainerList_;
    std::vector<TrainerTab> trainerTabs_;

    // ---- Equipment set state ----
    struct EquipmentSet {
        uint64_t setGuid = 0;
        uint32_t setId = 0;
        std::string name;
        std::string iconName;
        uint32_t ignoreSlotMask = 0;
        std::array<uint64_t, 19> itemGuids{};
    };
    std::vector<EquipmentSet> equipmentSets_;
    std::string pendingSaveSetName_;
    std::string pendingSaveSetIcon_;
    std::vector<EquipmentSetInfo> equipmentSetInfo_;

    // ---- Repair cost DBC cache ----
    mutable bool repairDbcLoaded_ = false;
    // DurabilityCosts.dbc: [itemLevel] -> multiplier[29] (weapon subclass 0-20, armor subclass+21)
    mutable std::unordered_map<uint32_t, std::array<uint32_t, 29>> durabilityCosts_;
    // DurabilityQuality.dbc: [id] -> quality_mod float
    mutable std::unordered_map<uint32_t, float> durabilityQuality_;
    void loadRepairDbc() const;
    uint32_t estimateItemRepairCost(uint64_t itemGuid) const;
};

} // namespace game
} // namespace wowee
