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
#include <utility>
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
    void selectGossipOption(uint32_t optionId, const std::string& code = "");
    void selectGossipQuest(uint32_t questId);
    void acceptQuest();
    /// `announce` fires QUEST_FINISHED, which is what closes FrameXML's
    /// quest frame. False when the interface is the one closing it.
    void declineQuest(bool announce = true);
    /// Clear the open detail page and tell the interface it is done, so the
    /// quest frame closes. Accepting a quest dismisses the page the same as
    /// declining it does - the interface only closes on QUEST_FINISHED, and
    /// the accept path used to reset the state without firing it, so this
    /// client's own window shut and FrameXML's stayed open.
    void dismissQuestDetails(bool announce = true);
    void closeGossip();
    void offerQuestFromItem(uint64_t itemGuid, uint32_t questId);

    [[nodiscard]] bool isGossipWindowOpen() const { return gossipWindowOpen_; }
    [[nodiscard]] const GossipMessageData& getCurrentGossip() const { return currentGossip_; }
    [[nodiscard]] const std::string& getNpcText(uint32_t textId) const;
    /// What a quest giver says over its list of quests, from
    /// SMSG_QUESTGIVER_QUEST_LIST. Empty when the window came from gossip
    /// instead, which carries its text as an npc-text id rather than inline.
    [[nodiscard]] const std::string& getQuestGreeting() const { return questGreeting_; }

    // Quest details
    /// Whether a quest giver's detail page is up. True from the moment the
    /// packet lands.
    ///
    /// It used to stay false for a hundred milliseconds after the details
    /// arrived, to give the item queries fired alongside them time to come
    /// back. That delay belonged to *this* client's own window, which draws
    /// reward icons and wants their names first - but it gated every reader,
    /// and the interface asks the instant it is told.
    ///
    /// What that cost: QUEST_DETAIL fires as the packet is handled, FrameXML
    /// runs QuestInfo_Display, and QuestInfo_ShowRewards asks how many rewards
    /// there are. For a hundred milliseconds the answer was none - so it hid
    /// the rewards block and returned nil, and a nil return is what tells
    /// QuestInfo_Display not to reparent it. The delay then expired, something
    /// called QuestInfo_ShowRewards directly, and it filled the block in and
    /// showed it - still parented to UIParent, still at the position its XML
    /// gave it. The reward list appeared in the middle of the screen, outside
    /// the quest frame, and closing the frame did not take it away because it
    /// was never inside it.
    [[nodiscard]] bool isQuestDetailsOpen() const { return questDetailsOpen_; }

    /// Whether the item queries sent when the details arrived have had time to
    /// answer. Only this client's own quest window asks: it draws the reward
    /// icons itself and they are blank until the names land. The interface
    /// does not, because FrameXML redraws when the item info event arrives.
    [[nodiscard]] bool questDetailsItemInfoReady() const {
        return questDetailsOpenTime_ == std::chrono::steady_clock::time_point{} ||
               std::chrono::steady_clock::now() >= questDetailsOpenTime_;
    }
    [[nodiscard]] const QuestDetailsData& getQuestDetails() const { return currentQuestDetails_; }

    // Gossip / quest map POI markers (aliased from handler_types.hpp)
    using GossipPoi = game::GossipPoi;
    [[nodiscard]] const std::vector<GossipPoi>& getGossipPois() const { return gossipPois_; }
    void clearGossipPois() { gossipPois_.clear(); }

    // Quest turn-in
    [[nodiscard]] bool isQuestRequestItemsOpen() const { return questRequestItemsOpen_; }
    [[nodiscard]] const QuestRequestItemsData& getQuestRequestItems() const { return currentQuestRequestItems_; }
    void completeQuest();
    void closeQuestRequestItems(bool announce = true);

    [[nodiscard]] bool isQuestOfferRewardOpen() const { return questOfferRewardOpen_; }
    [[nodiscard]] const QuestOfferRewardData& getQuestOfferReward() const { return currentQuestOfferReward_; }
    void chooseQuestReward(uint32_t rewardIndex);
    void closeQuestOfferReward(bool announce = true);

    // Quest log
    struct QuestLogEntry {
        uint32_t questId = 0;
        std::string title;
        std::string objectives;
        /// What the quest giver says, drawn above the objectives in the log.
        std::string description;
        /// Shown once every objective is done, in place of the objective list.
        std::string completionText;
        /// The spell the quest offers as a reward, or zero.
        uint32_t rewardSpellId = 0;
        uint32_t rewardXPId = 0;  // QuestXP.dbc column; reward XP = that column at this quest's level
        uint32_t rewardHonor = 0;
        uint32_t rewardTalents = 0;
        uint32_t rewardArenaPoints = 0;
        uint32_t rewardTitleId = 0;  // CharTitles.dbc id of an awarded title, 0 = none
        struct FactionReward { uint32_t factionId = 0; int32_t valueId = 0; int32_t override = 0; };
        std::array<FactionReward, 5> factionRewards{};
        int32_t level = 0;   // quest level from query response; 0 = unknown, -1 = player-scaling
        // ZoneOrSort from query response: >0 = AreaTable zone id, <0 = QuestSort.dbc
        // category (negated), 0 = unknown
        int32_t zoneOrSort = 0;
        bool complete = false;
        /// A timed quest that ran out, or one the server failed. Read from the
        /// bit beside `complete` in the same quest-slot field.
        bool failed = false;
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
        // The quest's start item - "SrcItemId" on the wire - which is the
        // usable item some quests hand you and the watch frame draws a button
        // for. Zero when the quest has none, which is most of them.
        uint32_t sourceItemId = 0;
        int32_t  rewardMoney = 0;
        std::array<QuestRewardItem, 4> rewardItems{};
        std::array<QuestRewardItem, 6> rewardChoiceItems{};
    };
    [[nodiscard]] const std::vector<QuestLogEntry>& getQuestLog() const { return questLog_; }
    /// Mutable access for the headless harness to seed a quest with text the
    /// server would otherwise supply, so the quest-log flow can be driven and
    /// drawn without a running realm. Not used on the live path.
    std::vector<QuestLogEntry>& questLogRef() { return questLog_; }

    /// Where a quest sits in the log, counting from one, or zero if it is not
    /// in it. This is what the interface means by a quest index - every quest
    /// log API takes it, and QUEST_WATCH_UPDATE carries it - and it is not the
    /// quest id, which is what was being sent in its place.
    [[nodiscard]] int questLogIndexOf(uint32_t questId) const {
        for (size_t i = 0; i < questLog_.size(); ++i) {
            if (questLog_[i].questId == questId) return static_cast<int>(i) + 1;
        }
        return 0;
    }
    // Seconds left on every timed quest, paired with its quest id, in quest log
    // order. The last field of each PLAYER_QUEST_LOG slot is when the quest
    // fails, as a Unix timestamp; zero means the quest is not timed.
    std::vector<std::pair<uint32_t, uint32_t>> getQuestTimers() const;
    // Server-side quest log capacity: 20 slots in Vanilla/Turtle, 25 from TBC on
    [[nodiscard]] int maxQuestLogSlots() const;
    // QuestSort.dbc name ("Seasonal", class/profession sorts, ...) for negative
    // ZoneOrSort values; empty if unknown
    const std::string& getQuestSortName(uint32_t sortId);

    /// The reward experience a quest shows: QuestXP.dbc read at the quest's
    /// level, in the column its XP-difficulty index selects. Zero when the
    /// level or index is unknown (index 0 is genuinely no XP), so the reward
    /// panel simply omits the line, as it does for a quest that gives none.
    uint32_t getQuestRewardXP(int32_t level, uint32_t xpDifficulty);

    /// The reputation a quest reward shows for one faction entry: the override
    /// in hundredths when set, otherwise QuestFactionReward.dbc read at the
    /// value index (row 1 for a gain, row 2 for a loss), the same as
    /// AzerothCore's Player::RewardReputation display value.
    int32_t getQuestRewardReputation(int32_t valueId, int32_t override);
    [[nodiscard]] int getSelectedQuestLogIndex() const { return selectedQuestLogIndex_; }
    void setSelectedQuestLogIndex(int idx) { selectedQuestLogIndex_ = idx; }
    void abandonQuest(uint32_t questId);
    void shareQuestWithParty(uint32_t questId);
    bool requestQuestQuery(uint32_t questId, bool force = false);
    // Tracker membership deliberately lives on GameHandler, not here: it is
    // what the HUD, the Lua watch bindings and the saved character config all
    // read, and this handler's own quest code reaches it via
    // owner_.setQuestTracked(). A second copy here would answer only to
    // whoever wrote it.
    [[nodiscard]] bool isQuestQueryPending(uint32_t questId) const {
        return pendingQuestQueryIds_.count(questId) > 0;
    }
    void clearQuestQueryPending(uint32_t questId) { pendingQuestQueryIds_.erase(questId); }

    // Quest giver status (! and ? markers)
    [[nodiscard]] QuestGiverStatus getQuestGiverStatus(uint64_t guid) const;
    [[nodiscard]] const std::unordered_map<uint64_t, QuestGiverStatus>& getNpcQuestStatuses() const { return npcQuestStatus_; }

    // Shared quest
    [[nodiscard]] bool hasPendingSharedQuest() const { return pendingSharedQuest_; }
    [[nodiscard]] uint32_t getSharedQuestId() const { return sharedQuestId_; }
    [[nodiscard]] const std::string& getSharedQuestTitle() const { return sharedQuestTitle_; }
    [[nodiscard]] const std::string& getSharedQuestSharerName() const { return sharedQuestSharerName_; }
    void acceptSharedQuest();
    void declineSharedQuest();

    // --- Internal helpers called from GameHandler ---
    [[nodiscard]] bool hasQuestInLog(uint32_t questId) const;
    [[nodiscard]] int findQuestLogSlotIndexFromServer(uint32_t questId) const;
    void addQuestToLocalLogIfMissing(uint32_t questId, const std::string& title, const std::string& objectives);
    bool resyncQuestLogFromServerSlots(bool forceQueryMetadata);
    void applyQuestStateFromFields(const FlatFieldMap& fields);
    void applyPackedKillCountsFromFields(QuestLogEntry& quest);
    // Reconcile collect-item objective progress against the player's actual
    // bag contents. In 3.3.5a the server does not push item objective counts
    // (unlike kill credit, which arrives packed in the quest-log update
    // fields) - the authentic client derives them by counting matching item
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
    /// Tell the interface a quest moved - see the definition for why the
    /// three events go together.
    void announceQuestLogChanged(uint32_t questId);

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
    std::string questGreeting_;
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
    // QuestXP.dbc, keyed by level; each row is the ten difficulty columns.
    std::unordered_map<int32_t, std::array<uint32_t, 10>> questXpByLevel_;
    bool questXpDbcLoaded_ = false;
    // QuestFactionReward.dbc, keyed by row id (1 = gains, 2 = losses); each row
    // is the ten value columns the reputation index reads.
    std::unordered_map<int32_t, std::array<int32_t, 10>> questFactionRew_;
    bool questFactionRewDbcLoaded_ = false;

    std::unordered_map<uint32_t, float> pendingQuestAcceptTimeouts_;
    std::unordered_map<uint32_t, uint64_t> pendingQuestAcceptNpcGuids_;
    bool questOfferRewardOpen_ = false;
    QuestOfferRewardData currentQuestOfferReward_;

    // Quest log
    std::vector<QuestLogEntry> questLog_;
    int selectedQuestLogIndex_ = 0;
    std::unordered_set<uint32_t> pendingQuestQueryIds_;

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
