#pragma once

#include "game/world_packets.hpp"
#include "game/character.hpp"
#include "game/inventory.hpp"
#include "game/spell_defines.hpp"
#include "game/group_defines.hpp"
#include <memory>
#include <string>
#include <vector>
#include <deque>
#include <array>
#include <functional>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>

namespace wowee {
namespace network { class WorldSocket; class Packet; }

namespace game {

/**
 * Quest giver status values (WoW 3.3.5a)
 */
enum class QuestGiverStatus : uint8_t {
    NONE = 0,
    UNAVAILABLE = 1,
    INCOMPLETE = 5,    // ? (gray)
    REWARD_REP = 6,
    AVAILABLE_LOW = 7, // ! (gray, low-level)
    AVAILABLE = 8,     // ! (yellow)
    REWARD = 10        // ? (yellow)
};

/**
 * World connection state
 */
enum class WorldState {
    DISCONNECTED,           // Not connected
    CONNECTING,             // TCP connection in progress
    CONNECTED,              // Connected, waiting for challenge
    CHALLENGE_RECEIVED,     // Received SMSG_AUTH_CHALLENGE
    AUTH_SENT,              // Sent CMSG_AUTH_SESSION, encryption initialized
    AUTHENTICATED,          // Received SMSG_AUTH_RESPONSE success
    READY,                  // Ready for character/world operations
    CHAR_LIST_REQUESTED,    // CMSG_CHAR_ENUM sent
    CHAR_LIST_RECEIVED,     // SMSG_CHAR_ENUM received
    ENTERING_WORLD,         // CMSG_PLAYER_LOGIN sent
    IN_WORLD,               // In game world
    FAILED                  // Connection or authentication failed
};

/**
 * World connection callbacks
 */
using WorldConnectSuccessCallback = std::function<void()>;
using WorldConnectFailureCallback = std::function<void(const std::string& reason)>;

/**
 * GameHandler - Manages world server connection and game protocol
 *
 * Handles:
 * - Connection to world server
 * - Authentication with session key from auth server
 * - RC4 header encryption
 * - Character enumeration
 * - World entry
 * - Game packets
 */
class GameHandler {
public:
    GameHandler();
    ~GameHandler();

    /**
     * Connect to world server
     *
     * @param host World server hostname/IP
     * @param port World server port (default 8085)
     * @param sessionKey 40-byte session key from auth server
     * @param accountName Account name (will be uppercased)
     * @param build Client build number (default 12340 for 3.3.5a)
     * @return true if connection initiated
     */
    bool connect(const std::string& host,
                 uint16_t port,
                 const std::vector<uint8_t>& sessionKey,
                 const std::string& accountName,
                 uint32_t build = 12340);

    /**
     * Disconnect from world server
     */
    void disconnect();

    /**
     * Check if connected to world server
     */
    bool isConnected() const;

    /**
     * Get current connection state
     */
    WorldState getState() const { return state; }

    /**
     * Request character list from server
     * Must be called when state is READY or AUTHENTICATED
     */
    void requestCharacterList();

    /**
     * Get list of characters (available after CHAR_LIST_RECEIVED state)
     */
    const std::vector<Character>& getCharacters() const { return characters; }

    void createCharacter(const CharCreateData& data);
    void deleteCharacter(uint64_t characterGuid);

    using CharCreateCallback = std::function<void(bool success, const std::string& message)>;
    void setCharCreateCallback(CharCreateCallback cb) { charCreateCallback_ = std::move(cb); }

    using CharDeleteCallback = std::function<void(bool success)>;
    void setCharDeleteCallback(CharDeleteCallback cb) { charDeleteCallback_ = std::move(cb); }
    uint8_t getLastCharDeleteResult() const { return lastCharDeleteResult_; }

    /**
     * Select and log in with a character
     * @param characterGuid GUID of character to log in with
     */
    void selectCharacter(uint64_t characterGuid);
    void setActiveCharacterGuid(uint64_t guid) { activeCharacterGuid_ = guid; }
    uint64_t getActiveCharacterGuid() const { return activeCharacterGuid_; }
    const Character* getActiveCharacter() const;
    const Character* getFirstCharacter() const;

    /**
     * Get current player movement info
     */
    const MovementInfo& getMovementInfo() const { return movementInfo; }

    /**
     * Send a movement packet
     * @param opcode Movement opcode (CMSG_MOVE_START_FORWARD, etc.)
     */
    void sendMovement(Opcode opcode);

    /**
     * Update player position
     * @param x X coordinate
     * @param y Y coordinate
     * @param z Z coordinate
     */
    void setPosition(float x, float y, float z);

    /**
     * Update player orientation
     * @param orientation Facing direction in radians
     */
    void setOrientation(float orientation);

    /**
     * Get entity manager (for accessing entities in view)
     */
    EntityManager& getEntityManager() { return entityManager; }
    const EntityManager& getEntityManager() const { return entityManager; }

    /**
     * Send a chat message
     * @param type Chat type (SAY, YELL, WHISPER, etc.)
     * @param message Message text
     * @param target Target name (for whispers, empty otherwise)
     */
    void sendChatMessage(ChatType type, const std::string& message, const std::string& target = "");

    /**
     * Get chat history (recent messages)
     * @param maxMessages Maximum number of messages to return (0 = all)
     * @return Vector of chat messages
     */
    const std::deque<MessageChatData>& getChatHistory() const { return chatHistory; }

    /**
     * Add a locally-generated chat message (e.g., emote feedback)
     */
    void addLocalChatMessage(const MessageChatData& msg);

    // Money (copper)
    uint64_t getMoneyCopper() const { return playerMoneyCopper_; }

    // Inventory
    Inventory& getInventory() { return inventory; }
    const Inventory& getInventory() const { return inventory; }
    bool consumeOnlineEquipmentDirty() { bool d = onlineEquipDirty_; onlineEquipDirty_ = false; return d; }

    // Targeting
    void setTarget(uint64_t guid);
    void clearTarget();
    uint64_t getTargetGuid() const { return targetGuid; }
    std::shared_ptr<Entity> getTarget() const;
    bool hasTarget() const { return targetGuid != 0; }
    void tabTarget(float playerX, float playerY, float playerZ);

    // Inspection
    void inspectTarget();

    // Server info commands
    void queryServerTime();
    void requestPlayedTime();
    void queryWho(const std::string& playerName = "");

    // Social commands
    void addFriend(const std::string& playerName, const std::string& note = "");
    void removeFriend(const std::string& playerName);
    void setFriendNote(const std::string& playerName, const std::string& note);
    void addIgnore(const std::string& playerName);
    void removeIgnore(const std::string& playerName);

    // Random roll
    void randomRoll(uint32_t minRoll = 1, uint32_t maxRoll = 100);

    // Logout commands
    void requestLogout();
    void cancelLogout();

    // Stand state
    void setStandState(uint8_t state);  // 0=stand, 1=sit, 2=sit_chair, 3=sleep, 4=sit_low_chair, 5=sit_medium_chair, 6=sit_high_chair, 7=dead, 8=kneel, 9=submerged

    // Display toggles
    void toggleHelm();
    void toggleCloak();

    // Follow/Assist
    void followTarget();
    void assistTarget();

    // PvP
    void togglePvp();

    // Guild commands
    void requestGuildInfo();
    void requestGuildRoster();
    void setGuildMotd(const std::string& motd);
    void promoteGuildMember(const std::string& playerName);
    void demoteGuildMember(const std::string& playerName);
    void leaveGuild();
    void inviteToGuild(const std::string& playerName);

    // Ready check
    void initiateReadyCheck();
    void respondToReadyCheck(bool ready);

    // Duel
    void forfeitDuel();

    // ---- Phase 1: Name queries ----
    void queryPlayerName(uint64_t guid);
    void queryCreatureInfo(uint32_t entry, uint64_t guid);
    std::string getCachedPlayerName(uint64_t guid) const;
    std::string getCachedCreatureName(uint32_t entry) const;

    // ---- Phase 2: Combat ----
    void startAutoAttack(uint64_t targetGuid);
    void stopAutoAttack();
    bool isAutoAttacking() const { return autoAttacking; }
    bool isAggressiveTowardPlayer(uint64_t guid) const { return hostileAttackers_.count(guid) > 0; }
    const std::vector<CombatTextEntry>& getCombatText() const { return combatText; }
    void updateCombatText(float deltaTime);

    // ---- Phase 3: Spells ----
    void castSpell(uint32_t spellId, uint64_t targetGuid = 0);
    void cancelCast();
    void cancelAura(uint32_t spellId);
    const std::vector<uint32_t>& getKnownSpells() const { return knownSpells; }
    bool isCasting() const { return casting; }
    uint32_t getCurrentCastSpellId() const { return currentCastSpellId; }
    float getCastProgress() const { return castTimeTotal > 0 ? (castTimeTotal - castTimeRemaining) / castTimeTotal : 0.0f; }
    float getCastTimeRemaining() const { return castTimeRemaining; }

    // Action bar
    static constexpr int ACTION_BAR_SLOTS = 12;
    std::array<ActionBarSlot, ACTION_BAR_SLOTS>& getActionBar() { return actionBar; }
    const std::array<ActionBarSlot, ACTION_BAR_SLOTS>& getActionBar() const { return actionBar; }
    void setActionBarSlot(int slot, ActionBarSlot::Type type, uint32_t id);

    // Auras
    const std::vector<AuraSlot>& getPlayerAuras() const { return playerAuras; }
    const std::vector<AuraSlot>& getTargetAuras() const { return targetAuras; }

    // NPC death callback (for animations)
    using NpcDeathCallback = std::function<void(uint64_t guid)>;
    void setNpcDeathCallback(NpcDeathCallback cb) { npcDeathCallback_ = std::move(cb); }

    // NPC respawn callback (health 0 → >0, resets animation to idle)
    using NpcRespawnCallback = std::function<void(uint64_t guid)>;
    void setNpcRespawnCallback(NpcRespawnCallback cb) { npcRespawnCallback_ = std::move(cb); }

    // Melee swing callback (for driving animation/SFX)
    using MeleeSwingCallback = std::function<void()>;
    void setMeleeSwingCallback(MeleeSwingCallback cb) { meleeSwingCallback_ = std::move(cb); }

    // NPC swing callback (plays attack animation on NPC)
    using NpcSwingCallback = std::function<void(uint64_t guid)>;
    void setNpcSwingCallback(NpcSwingCallback cb) { npcSwingCallback_ = std::move(cb); }

    // XP tracking
    uint32_t getPlayerXp() const { return playerXp_; }
    uint32_t getPlayerNextLevelXp() const { return playerNextLevelXp_; }
    uint32_t getPlayerLevel() const { return serverPlayerLevel_; }
    static uint32_t killXp(uint32_t playerLevel, uint32_t victimLevel);

    // World entry callback (online mode - triggered when entering world)
    // Parameters: mapId, x, y, z (canonical WoW coordinates)
    using WorldEntryCallback = std::function<void(uint32_t mapId, float x, float y, float z)>;
    void setWorldEntryCallback(WorldEntryCallback cb) { worldEntryCallback_ = std::move(cb); }

    // Creature spawn callback (online mode - triggered when creature enters view)
    // Parameters: guid, displayId, x, y, z (canonical), orientation
    using CreatureSpawnCallback = std::function<void(uint64_t guid, uint32_t displayId, float x, float y, float z, float orientation)>;
    void setCreatureSpawnCallback(CreatureSpawnCallback cb) { creatureSpawnCallback_ = std::move(cb); }

    // Creature despawn callback (online mode - triggered when creature leaves view)
    using CreatureDespawnCallback = std::function<void(uint64_t guid)>;
    void setCreatureDespawnCallback(CreatureDespawnCallback cb) { creatureDespawnCallback_ = std::move(cb); }

    // Faction hostility map (populated from FactionTemplate.dbc by Application)
    void setFactionHostileMap(std::unordered_map<uint32_t, bool> map) { factionHostileMap_ = std::move(map); }

    // Creature move callback (online mode - triggered by SMSG_MONSTER_MOVE)
    // Parameters: guid, x, y, z (canonical), duration_ms (0 = instant)
    using CreatureMoveCallback = std::function<void(uint64_t guid, float x, float y, float z, uint32_t durationMs)>;
    void setCreatureMoveCallback(CreatureMoveCallback cb) { creatureMoveCallback_ = std::move(cb); }

    // Cooldowns
    float getSpellCooldown(uint32_t spellId) const;

    // Player GUID
    uint64_t getPlayerGuid() const { return playerGuid; }
    void setPlayerGuid(uint64_t guid) { playerGuid = guid; }

    // Player death state
    bool isPlayerDead() const { return playerDead_; }
    void releaseSpirit();

    // ---- Phase 4: Group ----
    void inviteToGroup(const std::string& playerName);
    void acceptGroupInvite();
    void declineGroupInvite();
    void leaveGroup();
    bool isInGroup() const { return !partyData.isEmpty(); }
    const GroupListData& getPartyData() const { return partyData; }
    bool hasPendingGroupInvite() const { return pendingGroupInvite; }
    const std::string& getPendingInviterName() const { return pendingInviterName; }

    // ---- Phase 5: Loot ----
    void lootTarget(uint64_t guid);
    void lootItem(uint8_t slotIndex);
    void closeLoot();
    void activateSpiritHealer(uint64_t npcGuid);
    bool isLootWindowOpen() const { return lootWindowOpen; }
    const LootResponseData& getCurrentLoot() const { return currentLoot; }

    // NPC Gossip
    void interactWithNpc(uint64_t guid);
    void selectGossipOption(uint32_t optionId);
    void selectGossipQuest(uint32_t questId);
    void acceptQuest();
    void declineQuest();
    void closeGossip();
    bool isGossipWindowOpen() const { return gossipWindowOpen; }
    const GossipMessageData& getCurrentGossip() const { return currentGossip; }
    bool isQuestDetailsOpen() const { return questDetailsOpen; }
    const QuestDetailsData& getQuestDetails() const { return currentQuestDetails; }

    // Quest turn-in
    bool isQuestRequestItemsOpen() const { return questRequestItemsOpen_; }
    const QuestRequestItemsData& getQuestRequestItems() const { return currentQuestRequestItems_; }
    void completeQuest();       // Send CMSG_QUESTGIVER_COMPLETE_QUEST
    void closeQuestRequestItems();

    bool isQuestOfferRewardOpen() const { return questOfferRewardOpen_; }
    const QuestOfferRewardData& getQuestOfferReward() const { return currentQuestOfferReward_; }
    void chooseQuestReward(uint32_t rewardIndex);  // Send CMSG_QUESTGIVER_CHOOSE_REWARD
    void closeQuestOfferReward();

    // Quest log
    struct QuestLogEntry {
        uint32_t questId = 0;
        std::string title;
        std::string objectives;
        bool complete = false;
    };
    const std::vector<QuestLogEntry>& getQuestLog() const { return questLog_; }
    void abandonQuest(uint32_t questId);

    // Quest giver status (! and ? markers)
    QuestGiverStatus getQuestGiverStatus(uint64_t guid) const {
        auto it = npcQuestStatus_.find(guid);
        return (it != npcQuestStatus_.end()) ? it->second : QuestGiverStatus::NONE;
    }
    const std::unordered_map<uint64_t, QuestGiverStatus>& getNpcQuestStatuses() const { return npcQuestStatus_; }

    // Vendor
    void openVendor(uint64_t npcGuid);
    void closeVendor();
    void buyItem(uint64_t vendorGuid, uint32_t itemId, uint32_t slot, uint8_t count);
    void sellItem(uint64_t vendorGuid, uint64_t itemGuid, uint32_t count);
    void sellItemBySlot(int backpackIndex);
    void autoEquipItemBySlot(int backpackIndex);
    void useItemBySlot(int backpackIndex);
    void useItemById(uint32_t itemId);
    bool isVendorWindowOpen() const { return vendorWindowOpen; }
    const ListInventoryData& getVendorItems() const { return currentVendorItems; }
    const ItemQueryResponseData* getItemInfo(uint32_t itemId) const {
        auto it = itemInfoCache_.find(itemId);
        return (it != itemInfoCache_.end()) ? &it->second : nullptr;
    }
    uint64_t getBackpackItemGuid(int index) const {
        if (index < 0 || index >= static_cast<int>(backpackSlotGuids_.size())) return 0;
        return backpackSlotGuids_[index];
    }
    uint64_t getVendorGuid() const { return currentVendorItems.vendorGuid; }

    /**
     * Set callbacks
     */
    void setOnSuccess(WorldConnectSuccessCallback callback) { onSuccess = callback; }
    void setOnFailure(WorldConnectFailureCallback callback) { onFailure = callback; }

    /**
     * Update - call regularly (e.g., each frame)
     *
     * @param deltaTime Time since last update in seconds
     */
    void update(float deltaTime);

private:
    /**
     * Handle incoming packet from world server
     */
    void handlePacket(network::Packet& packet);

    /**
     * Handle SMSG_AUTH_CHALLENGE from server
     */
    void handleAuthChallenge(network::Packet& packet);

    /**
     * Handle SMSG_AUTH_RESPONSE from server
     */
    void handleAuthResponse(network::Packet& packet);

    /**
     * Handle SMSG_CHAR_ENUM from server
     */
    void handleCharEnum(network::Packet& packet);

    /**
     * Handle SMSG_LOGIN_VERIFY_WORLD from server
     */
    void handleLoginVerifyWorld(network::Packet& packet);

    /**
     * Handle SMSG_ACCOUNT_DATA_TIMES from server
     */
    void handleAccountDataTimes(network::Packet& packet);

    /**
     * Handle SMSG_MOTD from server
     */
    void handleMotd(network::Packet& packet);

    /**
     * Handle SMSG_PONG from server
     */
    void handlePong(network::Packet& packet);

    /**
     * Handle SMSG_UPDATE_OBJECT from server
     */
    void handleUpdateObject(network::Packet& packet);

    /**
     * Handle SMSG_COMPRESSED_UPDATE_OBJECT from server
     */
    void handleCompressedUpdateObject(network::Packet& packet);

    /**
     * Handle SMSG_DESTROY_OBJECT from server
     */
    void handleDestroyObject(network::Packet& packet);

    /**
     * Handle SMSG_MESSAGECHAT from server
     */
    void handleMessageChat(network::Packet& packet);

    // ---- Phase 1 handlers ----
    void handleNameQueryResponse(network::Packet& packet);
    void handleCreatureQueryResponse(network::Packet& packet);
    void handleItemQueryResponse(network::Packet& packet);
    void queryItemInfo(uint32_t entry, uint64_t guid);
    void rebuildOnlineInventory();
    void detectInventorySlotBases(const std::map<uint16_t, uint32_t>& fields);
    bool applyInventoryFields(const std::map<uint16_t, uint32_t>& fields);
    uint64_t resolveOnlineItemGuid(uint32_t itemId) const;

    // ---- Phase 2 handlers ----
    void handleAttackStart(network::Packet& packet);
    void handleAttackStop(network::Packet& packet);
    void handleAttackerStateUpdate(network::Packet& packet);
    void handleSpellDamageLog(network::Packet& packet);
    void handleSpellHealLog(network::Packet& packet);

    // ---- Phase 3 handlers ----
    void handleInitialSpells(network::Packet& packet);
    void handleCastFailed(network::Packet& packet);
    void handleSpellStart(network::Packet& packet);
    void handleSpellGo(network::Packet& packet);
    void handleSpellCooldown(network::Packet& packet);
    void handleCooldownEvent(network::Packet& packet);
    void handleAuraUpdate(network::Packet& packet, bool isAll);
    void handleLearnedSpell(network::Packet& packet);
    void handleRemovedSpell(network::Packet& packet);

    // ---- Phase 4 handlers ----
    void handleGroupInvite(network::Packet& packet);
    void handleGroupDecline(network::Packet& packet);
    void handleGroupList(network::Packet& packet);
    void handleGroupUninvite(network::Packet& packet);
    void handlePartyCommandResult(network::Packet& packet);

    // ---- Character creation handler ----
    void handleCharCreateResponse(network::Packet& packet);

    // ---- XP handler ----
    void handleXpGain(network::Packet& packet);

    // ---- Creature movement handler ----
    void handleMonsterMove(network::Packet& packet);

    // ---- Phase 5 handlers ----
    void handleLootResponse(network::Packet& packet);
    void handleLootReleaseResponse(network::Packet& packet);
    void handleLootRemoved(network::Packet& packet);
    void handleGossipMessage(network::Packet& packet);
    void handleGossipComplete(network::Packet& packet);
    void handleQuestDetails(network::Packet& packet);
    void handleQuestRequestItems(network::Packet& packet);
    void handleQuestOfferReward(network::Packet& packet);
    void handleListInventory(network::Packet& packet);
    void addMoneyCopper(uint32_t amount);

    // ---- Server info handlers ----
    void handleQueryTimeResponse(network::Packet& packet);
    void handlePlayedTime(network::Packet& packet);
    void handleWho(network::Packet& packet);

    // ---- Social handlers ----
    void handleFriendStatus(network::Packet& packet);
    void handleRandomRoll(network::Packet& packet);

    // ---- Logout handlers ----
    void handleLogoutResponse(network::Packet& packet);
    void handleLogoutComplete(network::Packet& packet);

    void addCombatText(CombatTextEntry::Type type, int32_t amount, uint32_t spellId, bool isPlayerSource);
    void addSystemChatMessage(const std::string& message);

    /**
     * Send CMSG_PING to server (heartbeat)
     */
    void sendPing();

    /**
     * Send CMSG_AUTH_SESSION to server
     */
    void sendAuthSession();

    /**
     * Generate random client seed
     */
    uint32_t generateClientSeed();

    /**
     * Change state with logging
     */
    void setState(WorldState newState);

    /**
     * Fail connection with reason
     */
    void fail(const std::string& reason);

    // Network
    std::unique_ptr<network::WorldSocket> socket;

    // State
    WorldState state = WorldState::DISCONNECTED;

    // Authentication data
    std::vector<uint8_t> sessionKey;    // 40-byte session key from auth server
    std::string accountName;             // Account name
    uint32_t build = 12340;              // Client build (3.3.5a)
    uint32_t clientSeed = 0;             // Random seed generated by client
    uint32_t serverSeed = 0;             // Seed from SMSG_AUTH_CHALLENGE

    // Characters
    std::vector<Character> characters;       // Character list from SMSG_CHAR_ENUM

    // Movement
    MovementInfo movementInfo;               // Current player movement state
    uint32_t movementTime = 0;               // Movement timestamp counter

    // Inventory
    Inventory inventory;

    // Entity tracking
    EntityManager entityManager;             // Manages all entities in view

    // Chat
    std::deque<MessageChatData> chatHistory;    // Recent chat messages
    size_t maxChatHistory = 100;             // Maximum chat messages to keep

    // Targeting
    uint64_t targetGuid = 0;
    std::vector<uint64_t> tabCycleList;
    int tabCycleIndex = -1;
    bool tabCycleStale = true;

    // Heartbeat
    uint32_t pingSequence = 0;               // Ping sequence number (increments)
    float timeSinceLastPing = 0.0f;          // Time since last ping sent (seconds)
    float pingInterval = 30.0f;              // Ping interval (30 seconds)
    uint32_t lastLatency = 0;                // Last measured latency (milliseconds)

    // Player GUID
    uint64_t playerGuid = 0;

    // ---- Phase 1: Name caches ----
    std::unordered_map<uint64_t, std::string> playerNameCache;
    std::unordered_set<uint64_t> pendingNameQueries;
    std::unordered_map<uint32_t, CreatureQueryResponseData> creatureInfoCache;
    std::unordered_set<uint32_t> pendingCreatureQueries;

    // ---- Friend list cache ----
    std::unordered_map<std::string, uint64_t> friendsCache;  // name -> guid

    // ---- Ignore list cache ----
    std::unordered_map<std::string, uint64_t> ignoreCache;  // name -> guid

    // ---- Logout state ----
    bool loggingOut_ = false;

    // ---- Display state ----
    bool helmVisible_ = true;
    bool cloakVisible_ = true;

    // ---- Follow state ----
    uint64_t followTargetGuid_ = 0;

    // ---- Online item tracking ----
    struct OnlineItemInfo {
        uint32_t entry = 0;
        uint32_t stackCount = 1;
    };
    std::unordered_map<uint64_t, OnlineItemInfo> onlineItems_;
    std::unordered_map<uint32_t, ItemQueryResponseData> itemInfoCache_;
    std::unordered_set<uint32_t> pendingItemQueries_;
    std::array<uint64_t, 23> equipSlotGuids_{};
    std::array<uint64_t, 16> backpackSlotGuids_{};
    int invSlotBase_ = -1;
    int packSlotBase_ = -1;
    std::map<uint16_t, uint32_t> lastPlayerFields_;
    bool onlineEquipDirty_ = false;

    // ---- Phase 2: Combat ----
    bool autoAttacking = false;
    uint64_t autoAttackTarget = 0;
    bool autoAttackOutOfRange_ = false;
    std::unordered_set<uint64_t> hostileAttackers_;
    std::vector<CombatTextEntry> combatText;

    // ---- Phase 3: Spells ----
    WorldEntryCallback worldEntryCallback_;
    CreatureSpawnCallback creatureSpawnCallback_;
    CreatureDespawnCallback creatureDespawnCallback_;
    CreatureMoveCallback creatureMoveCallback_;
    std::vector<uint32_t> knownSpells;
    std::unordered_map<uint32_t, float> spellCooldowns;    // spellId -> remaining seconds
    uint8_t castCount = 0;
    bool casting = false;
    uint32_t currentCastSpellId = 0;
    float castTimeRemaining = 0.0f;
    float castTimeTotal = 0.0f;
    std::array<ActionBarSlot, 12> actionBar{};
    std::vector<AuraSlot> playerAuras;
    std::vector<AuraSlot> targetAuras;

    // ---- Phase 4: Group ----
    GroupListData partyData;
    bool pendingGroupInvite = false;
    std::string pendingInviterName;

    uint64_t activeCharacterGuid_ = 0;

    // ---- Phase 5: Loot ----
    bool lootWindowOpen = false;
    LootResponseData currentLoot;
    struct LocalLootState {
        LootResponseData data;
        bool moneyTaken = false;
    };
    std::unordered_map<uint64_t, LocalLootState> localLootState_;
    uint64_t playerMoneyCopper_ = 0;

    // Gossip
    bool gossipWindowOpen = false;
    GossipMessageData currentGossip;

    // Quest details
    bool questDetailsOpen = false;
    QuestDetailsData currentQuestDetails;

    // Quest turn-in
    bool questRequestItemsOpen_ = false;
    QuestRequestItemsData currentQuestRequestItems_;
    bool questOfferRewardOpen_ = false;
    QuestOfferRewardData currentQuestOfferReward_;

    // Quest log
    std::vector<QuestLogEntry> questLog_;

    // Quest giver status per NPC
    std::unordered_map<uint64_t, QuestGiverStatus> npcQuestStatus_;

    // Faction hostility lookup (populated from FactionTemplate.dbc)
    std::unordered_map<uint32_t, bool> factionHostileMap_;
    bool isHostileFaction(uint32_t factionTemplateId) const {
        auto it = factionHostileMap_.find(factionTemplateId);
        return it != factionHostileMap_.end() ? it->second : true; // default hostile if unknown
    }

    // Vendor
    bool vendorWindowOpen = false;
    ListInventoryData currentVendorItems;

    // Callbacks
    WorldConnectSuccessCallback onSuccess;
    WorldConnectFailureCallback onFailure;
    CharCreateCallback charCreateCallback_;
    CharDeleteCallback charDeleteCallback_;
    uint8_t lastCharDeleteResult_ = 0xFF;
    bool pendingCharCreateResult_ = false;
    bool pendingCharCreateSuccess_ = false;
    std::string pendingCharCreateMsg_;

    // ---- XP tracking ----
    uint32_t playerXp_ = 0;
    uint32_t playerNextLevelXp_ = 0;
    uint32_t serverPlayerLevel_ = 1;
    static uint32_t xpForLevel(uint32_t level);

    NpcDeathCallback npcDeathCallback_;
    NpcRespawnCallback npcRespawnCallback_;
    MeleeSwingCallback meleeSwingCallback_;
    NpcSwingCallback npcSwingCallback_;
    bool playerDead_ = false;
};

} // namespace game
} // namespace wowee
