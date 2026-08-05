#pragma once

#include "game/world_packets.hpp"
#include "game/opcode_table.hpp"
#include "game/handler_types.hpp"
#include "network/packet.hpp"
#include <array>
#include <chrono>
#include <functional>
#include <map>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace wowee {
namespace game {

class GameHandler;
enum class QuestGiverStatus : uint8_t;

class QuestHandler {
public:
    using PacketHandler = std::function<void(network::Packet&)>;
    using DispatchTable = std::unordered_map<LogicalOpcode, PacketHandler>;

    explicit QuestHandler(GameHandler& owner);

    void registerOpcodes(DispatchTable& table);

    // --- Public API (delegated from GameHandler) ---

    // NPC Gossip
    void selectGossipOption(uint32_t optionId);
    void selectGossipQuest(uint32_t questId);
    void acceptQuest();
    void declineQuest();
    void closeGossip();
    void offerQuestFromItem(uint64_t itemGuid, uint32_t questId);

    bool isGossipWindowOpen() const { return gossipWindowOpen_; }
    const GossipMessageData& getCurrentGossip() const { return currentGossip_; }
    const std::string& getNpcText(uint32_t textId) const;

    // Quest details
    bool isQuestDetailsOpen() {
        if (questDetailsOpen_) return true;
        if (questDetailsOpenTime_ != std::chrono::steady_clock::time_point{}) {
            if (std::chrono::steady_clock::now() >= questDetailsOpenTime_) {
                questDetailsOpen_ = true;
                questDetailsOpenTime_ = std::chrono::steady_clock::time_point{};
                return true;
            }
        }
        return false;
    }
    const QuestDetailsData& getQuestDetails() const { return currentQuestDetails_; }

    // Gossip / quest map POI markers (aliased from handler_types.hpp)
    using GossipPoi = game::GossipPoi;
    const std::vector<GossipPoi>& getGossipPois() const { return gossipPois_; }
    void clearGossipPois() { gossipPois_.clear(); }

    // Quest turn-in
    bool isQuestRequestItemsOpen() const { return questRequestItemsOpen_; }
    const QuestRequestItemsData& getQuestRequestItems() const { return currentQuestRequestItems_; }
    void completeQuest();
    void closeQuestRequestItems();

    bool isQuestOfferRewardOpen() const { return questOfferRewardOpen_; }
    const QuestOfferRewardData& getQuestOfferReward() const { return currentQuestOfferReward_; }
    void chooseQuestReward(uint32_t rewardIndex);
    void closeQuestOfferReward();

    // Quest log
    struct QuestLogEntry {
        uint32_t questId = 0;
        std::string title;
        std::string objectives;
        int32_t level = 0;   // quest level from query response; 0 = unknown, -1 = player-scaling
        // ZoneOrSort from query response: >0 = AreaTable zone id, <0 = QuestSort.dbc
        // category (negated), 0 = unknown
        int32_t zoneOrSort = 0;
        bool complete = false;
        std::unordered_map<uint32_t, std::pair<uint32_t, uint32_t>> killCounts;
        std::unordered_map<uint32_t, uint32_t> itemCounts;
        std::unordered_map<uint32_t, uint32_t> requiredItemCounts;
        struct KillObjective {
            int32_t npcOrGoId = 0;
            uint32_t required = 0;
        };
        std::array<KillObjective, 4> killObjectives{};
        struct ItemObjective {
            uint32_t itemId = 0;
            uint32_t required = 0;
        };
        std::array<ItemObjective, 6> itemObjectives{};
        int32_t  rewardMoney = 0;
        std::array<QuestRewardItem, 4> rewardItems{};
        std::array<QuestRewardItem, 6> rewardChoiceItems{};
    };
    const std::vector<QuestLogEntry>& getQuestLog() const { return questLog_; }
    // Server-side quest log capacity: 20 slots in Vanilla/Turtle, 25 from TBC on
    int maxQuestLogSlots() const;
    // QuestSort.dbc name ("Seasonal", class/profession sorts, ...) for negative
    // ZoneOrSort values; empty if unknown
    const std::string& getQuestSortName(uint32_t sortId);
    int getSelectedQuestLogIndex() const { return selectedQuestLogIndex_; }
    void setSelectedQuestLogIndex(int idx) { selectedQuestLogIndex_ = idx; }
    void abandonQuest(uint32_t questId);
    void shareQuestWithParty(uint32_t questId);
    bool requestQuestQuery(uint32_t questId, bool force = false);
    bool isQuestTracked(uint32_t questId) const { return trackedQuestIds_.count(questId) > 0; }
    void setQuestTracked(uint32_t questId, bool tracked);
    const std::unordered_set<uint32_t>& getTrackedQuestIds() const { return trackedQuestIds_; }
    bool isQuestQueryPending(uint32_t questId) const {
        return pendingQuestQueryIds_.count(questId) > 0;
    }
    void clearQuestQueryPending(uint32_t questId) { pendingQuestQueryIds_.erase(questId); }

    // Quest giver status (! and ? markers)
    QuestGiverStatus getQuestGiverStatus(uint64_t guid) const;
    const std::unordered_map<uint64_t, QuestGiverStatus>& getNpcQuestStatuses() const { return npcQuestStatus_; }

    // Shared quest
    bool hasPendingSharedQuest() const { return pendingSharedQuest_; }
    uint32_t getSharedQuestId() const { return sharedQuestId_; }
    const std::string& getSharedQuestTitle() const { return sharedQuestTitle_; }
    const std::string& getSharedQuestSharerName() const { return sharedQuestSharerName_; }
    void acceptSharedQuest();
    void declineSharedQuest();

    // --- Internal helpers called from GameHandler ---
    bool hasQuestInLog(uint32_t questId) const;
    int findQuestLogSlotIndexFromServer(uint32_t questId) const;
    void addQuestToLocalLogIfMissing(uint32_t questId, const std::string& title, const std::string& objectives);
    bool resyncQuestLogFromServerSlots(bool forceQueryMetadata);
    void applyQuestStateFromFields(const FlatFieldMap& fields);
    void applyPackedKillCountsFromFields(QuestLogEntry& quest);
    // Reconcile collect-item objective progress against the player's actual
    // bag contents. In 3.3.5a the server does not push item objective counts
    // (unlike kill credit, which arrives packed in the quest-log update
    // fields) — the authentic client derives them by counting matching item
    // IDs in the bags. Called after every inventory rebuild. `carriedCounts`
    // maps itemId -> total quantity currently held across backpack + bags.
    void reconcileItemObjectivesFromInventory(
        const std::unordered_map<uint32_t, uint32_t>& carriedCounts);
    void clearPendingQuestAccept(uint32_t questId);
    void triggerQuestAcceptResync(uint32_t questId, uint64_t npcGuid, const char* reason);

    // Pending quest accept timeout state (used by GameHandler::update)
    std::unordered_map<uint32_t, float>& pendingQuestAcceptTimeoutsRef() { return pendingQuestAcceptTimeouts_; }
    std::unordered_map<uint32_t, uint64_t>& pendingQuestAcceptNpcGuidsRef() { return pendingQuestAcceptNpcGuids_; }
    // (login quest resync state lives in GameHandler, which drives its timing)

    // Direct state access for vendor/gossip interaction in GameHandler
    bool& gossipWindowOpenRef() { return gossipWindowOpen_; }
    GossipMessageData& currentGossipRef() { return currentGossip_; }
    std::unordered_map<uint64_t, QuestGiverStatus>& npcQuestStatusRef() { return npcQuestStatus_; }

    /// Everything this handler holds about the character that is leaving.
    ///
    /// The reset path used to clear GameHandler's same-named members, which
    /// are dead: every reader forwards here. So a character switch left the
    /// previous character's quest log and quest-giver marks in place until the
    /// server happened to overwrite them.
    void clearQuestStateForCharacterSwitch() {
        questLog_.clear();
        pendingQuestQueryIds_.clear();
        npcQuestStatus_.clear();
    }

    // Drives the quest-giver status requery cooldown; called once per frame by
    // GameHandler::update alongside the other quest bookkeeping.
    void tickQuestGiverStatusRequery(float deltaTime);

private:
    // Request fresh quest-giver status for nearby NPCs so the !/? markers update
    // live (e.g. the moment an objective completes) instead of only when the
    // player leaves and re-enters the area.
    //
    // Rate-limited rather than sent on the spot: one call fans out to a packet
    // per nearby quest giver, and completion events arrive in bursts (a killing
    // blow that finishes two quests, a loot that fills several objectives).
    // Requests inside the cooldown set a pending flag and are coalesced into a
    // single sweep when it expires, so no refresh is dropped.
    void requeryNearbyQuestGiverStatus();
    void sendQuestGiverStatusQueries();

    // Minimum spacing between requery sweeps. Marker freshness is a cosmetic
    // concern, so a second of latency is imperceptible next to the packet cost.
    static constexpr float kQuestGiverRequeryIntervalSec = 1.0f;
    float questGiverRequeryCooldown_ = 0.0f;
    bool questGiverRequeryPending_ = false;

    // --- Packet handlers ---
    void handleGossipMessage(network::Packet& packet);
    void handleQuestgiverQuestList(network::Packet& packet);
    void classifyGossipQuests(bool updateQuestLog);
    void handleGossipComplete(network::Packet& packet);
    void handleNpcTextUpdate(network::Packet& packet);
    void handleQuestPoiQueryResponse(network::Packet& packet);
    void handleQuestDetails(network::Packet& packet);
    void handleQuestRequestItems(network::Packet& packet);
    void handleQuestOfferReward(network::Packet& packet);
    void handleQuestConfirmAccept(network::Packet& packet);

    GameHandler& owner_;

    // --- State ---
    // Gossip
    bool gossipWindowOpen_ = false;
    GossipMessageData currentGossip_;
    std::vector<GossipPoi> gossipPois_;

    // Quest details
    bool questDetailsOpen_ = false;
    std::chrono::steady_clock::time_point questDetailsOpenTime_{};
    QuestDetailsData currentQuestDetails_;

    // Quest turn-in
    bool questRequestItemsOpen_ = false;
    QuestRequestItemsData currentQuestRequestItems_;
    uint32_t pendingTurnInQuestId_ = 0;
    uint64_t pendingTurnInNpcGuid_ = 0;
    bool pendingTurnInRewardRequest_ = false;
    // QuestSort.dbc names, loaded lazily
    std::unordered_map<uint32_t, std::string> questSortNames_;
    bool questSortDbcLoaded_ = false;

    std::unordered_map<uint32_t, float> pendingQuestAcceptTimeouts_;
    std::unordered_map<uint32_t, uint64_t> pendingQuestAcceptNpcGuids_;
    bool questOfferRewardOpen_ = false;
    QuestOfferRewardData currentQuestOfferReward_;

    // Quest log
    std::vector<QuestLogEntry> questLog_;
    int selectedQuestLogIndex_ = 0;
    std::unordered_set<uint32_t> pendingQuestQueryIds_;
    std::unordered_set<uint32_t> trackedQuestIds_;

    // Quest giver status per NPC
    std::unordered_map<uint64_t, QuestGiverStatus> npcQuestStatus_;

    // NPC gossip text cache (textId → body text)
    std::unordered_map<uint32_t, std::string> npcTextCache_;

    // Shared quest state
    bool        pendingSharedQuest_       = false;
    uint32_t    sharedQuestId_            = 0;
    std::string sharedQuestTitle_;
    std::string sharedQuestSharerName_;
    uint64_t    sharedQuestSharerGuid_    = 0;
};

} // namespace game
} // namespace wowee
