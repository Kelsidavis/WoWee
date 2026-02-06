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

    using CharCreateCallback = std::function<void(bool success, const std::string& message)>;
    void setCharCreateCallback(CharCreateCallback cb) { charCreateCallback_ = std::move(cb); }

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

    // Single-player: mark character list ready for selection UI
    void setSinglePlayerCharListReady();
    struct SinglePlayerSettings {
        bool fullscreen = false;
        bool vsync = true;
        bool shadows = true;
        int resWidth = 1920;
        int resHeight = 1080;
        int musicVolume = 30;
        int sfxVolume = 100;
        float mouseSensitivity = 0.2f;
        bool invertMouse = false;
    };
    bool getSinglePlayerSettings(SinglePlayerSettings& out) const;
    void setSinglePlayerSettings(const SinglePlayerSettings& settings);

    // Inventory
    Inventory& getInventory() { return inventory; }
    const Inventory& getInventory() const { return inventory; }

    // Targeting
    void setTarget(uint64_t guid);
    void clearTarget();
    uint64_t getTargetGuid() const { return targetGuid; }
    std::shared_ptr<Entity> getTarget() const;
    bool hasTarget() const { return targetGuid != 0; }
    void tabTarget(float playerX, float playerY, float playerZ);

    // ---- Phase 1: Name queries ----
    void queryPlayerName(uint64_t guid);
    void queryCreatureInfo(uint32_t entry, uint64_t guid);
    std::string getCachedPlayerName(uint64_t guid) const;
    std::string getCachedCreatureName(uint32_t entry) const;

    // ---- Phase 2: Combat ----
    void startAutoAttack(uint64_t targetGuid);
    void stopAutoAttack();
    bool isAutoAttacking() const { return autoAttacking; }
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

    // Single-player mode
    void setSinglePlayerMode(bool sp) { singlePlayerMode_ = sp; }
    bool isSinglePlayerMode() const { return singlePlayerMode_; }
    void simulateMotd(const std::vector<std::string>& lines);
    void applySinglePlayerStartData(Race race, Class cls);
    bool loadSinglePlayerCharacterState(uint64_t guid);
    void notifyInventoryChanged();
    void notifyEquipmentChanged();
    void notifyQuestStateChanged();
    void flushSinglePlayerSave();

    // NPC death callback (single-player)
    using NpcDeathCallback = std::function<void(uint64_t guid)>;
    void setNpcDeathCallback(NpcDeathCallback cb) { npcDeathCallback_ = std::move(cb); }

    // Melee swing callback (for driving animation/SFX)
    using MeleeSwingCallback = std::function<void()>;
    void setMeleeSwingCallback(MeleeSwingCallback cb) { meleeSwingCallback_ = std::move(cb); }

    // Local player stats (single-player)
    uint32_t getLocalPlayerHealth() const { return localPlayerHealth_; }
    uint32_t getLocalPlayerMaxHealth() const { return localPlayerMaxHealth_; }
    void initLocalPlayerStats(uint32_t level, uint32_t hp, uint32_t maxHp) {
        localPlayerLevel_ = level;
        localPlayerHealth_ = hp;
        localPlayerMaxHealth_ = maxHp;
        playerNextLevelXp_ = xpForLevel(level);
        playerXp_ = 0;
    }

    // XP tracking (works in both single-player and server modes)
    uint32_t getPlayerXp() const { return playerXp_; }
    uint32_t getPlayerNextLevelXp() const { return playerNextLevelXp_; }
    uint32_t getPlayerLevel() const { return singlePlayerMode_ ? localPlayerLevel_ : serverPlayerLevel_; }

    // Hearthstone callback (single-player teleport)
    using HearthstoneCallback = std::function<void()>;
    void setHearthstoneCallback(HearthstoneCallback cb) { hearthstoneCallback = std::move(cb); }

    // Cooldowns
    float getSpellCooldown(uint32_t spellId) const;

    // Player GUID
    uint64_t getPlayerGuid() const { return playerGuid; }
    void setPlayerGuid(uint64_t guid) { playerGuid = guid; }

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
    bool isLootWindowOpen() const { return lootWindowOpen; }
    const LootResponseData& getCurrentLoot() const { return currentLoot; }

    // NPC Gossip
    void interactWithNpc(uint64_t guid);
    void selectGossipOption(uint32_t optionId);
    void closeGossip();
    bool isGossipWindowOpen() const { return gossipWindowOpen; }
    const GossipMessageData& getCurrentGossip() const { return currentGossip; }

    // Vendor
    void openVendor(uint64_t npcGuid);
    void buyItem(uint64_t vendorGuid, uint32_t itemId, uint32_t slot, uint8_t count);
    void sellItem(uint64_t vendorGuid, uint64_t itemGuid, uint8_t count);
    bool isVendorWindowOpen() const { return vendorWindowOpen; }
    const ListInventoryData& getVendorItems() const { return currentVendorItems; }

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

    struct SinglePlayerCreateInfo {
        uint32_t mapId = 0;
        uint32_t zoneId = 0;
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        float orientation = 0.0f;
    };
    bool getSinglePlayerCreateInfo(Race race, Class cls, SinglePlayerCreateInfo& out) const;

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

    // ---- Phase 5 handlers ----
    void handleLootResponse(network::Packet& packet);
    void handleLootReleaseResponse(network::Packet& packet);
    void handleLootRemoved(network::Packet& packet);
    void handleGossipMessage(network::Packet& packet);
    void handleGossipComplete(network::Packet& packet);
    void handleListInventory(network::Packet& packet);
    LootResponseData generateLocalLoot(uint64_t guid);
    void simulateLootResponse(const LootResponseData& data);
    void simulateLootRelease();
    void simulateLootRemove(uint8_t slotIndex);
    void simulateXpGain(uint64_t victimGuid, uint32_t totalXp);
    void addMoneyCopper(uint32_t amount);

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

    // ---- Phase 2: Combat ----
    bool autoAttacking = false;
    uint64_t autoAttackTarget = 0;
    std::vector<CombatTextEntry> combatText;

    // ---- Phase 3: Spells ----
    HearthstoneCallback hearthstoneCallback;
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

    // Vendor
    bool vendorWindowOpen = false;
    ListInventoryData currentVendorItems;

    // Callbacks
    WorldConnectSuccessCallback onSuccess;
    WorldConnectFailureCallback onFailure;
    CharCreateCallback charCreateCallback_;
    bool pendingCharCreateResult_ = false;
    bool pendingCharCreateSuccess_ = false;
    std::string pendingCharCreateMsg_;

    // ---- XP tracking ----
    uint32_t playerXp_ = 0;
    uint32_t playerNextLevelXp_ = 0;
    uint32_t serverPlayerLevel_ = 1;
    void awardLocalXp(uint64_t victimGuid, uint32_t victimLevel);
    void levelUp();
    static uint32_t xpForLevel(uint32_t level);
    static uint32_t killXp(uint32_t playerLevel, uint32_t victimLevel);

    // ---- Single-player combat ----
    bool singlePlayerMode_ = false;
    float swingTimer_ = 0.0f;
    static constexpr float SWING_SPEED = 2.0f;
    NpcDeathCallback npcDeathCallback_;
    MeleeSwingCallback meleeSwingCallback_;
    uint32_t localPlayerHealth_ = 0;
    uint32_t localPlayerMaxHealth_ = 0;
    uint32_t localPlayerLevel_ = 1;

    struct NpcAggroEntry {
        uint64_t guid;
        float swingTimer;
    };
    std::vector<NpcAggroEntry> aggroList_;

    void updateLocalCombat(float deltaTime);
    void updateNpcAggro(float deltaTime);
    void performPlayerSwing();
    void performNpcSwing(uint64_t guid);
    void handleNpcDeath(uint64_t guid);
    void aggroNpc(uint64_t guid);
    bool isNpcAggroed(uint64_t guid) const;

    // ---- Single-player persistence ----
    enum SinglePlayerDirty : uint32_t {
        SP_DIRTY_NONE       = 0,
        SP_DIRTY_CHAR       = 1 << 0,
        SP_DIRTY_INVENTORY  = 1 << 1,
        SP_DIRTY_SPELLS     = 1 << 2,
        SP_DIRTY_ACTIONBAR  = 1 << 3,
        SP_DIRTY_AURAS      = 1 << 4,
        SP_DIRTY_QUESTS     = 1 << 5,
        SP_DIRTY_MONEY      = 1 << 6,
        SP_DIRTY_XP         = 1 << 7,
        SP_DIRTY_POSITION   = 1 << 8,
        SP_DIRTY_STATS      = 1 << 9,
        SP_DIRTY_SETTINGS   = 1 << 10,
        SP_DIRTY_ALL        = 0xFFFFFFFFu
    };
    void markSinglePlayerDirty(uint32_t flags, bool highPriority);
    void loadSinglePlayerCharacters();
    void saveSinglePlayerCharacterState(bool force);

    uint32_t spDirtyFlags_ = SP_DIRTY_NONE;
    bool spDirtyHighPriority_ = false;
    float spDirtyTimer_ = 0.0f;
    float spPeriodicTimer_ = 0.0f;
    float spLastDirtyX_ = 0.0f;
    float spLastDirtyY_ = 0.0f;
    float spLastDirtyZ_ = 0.0f;
    float spLastDirtyOrientation_ = 0.0f;
    std::unordered_map<uint64_t, bool> spHasState_;
    std::unordered_map<uint64_t, float> spSavedOrientation_;
    SinglePlayerSettings spSettings_{};
    bool spSettingsLoaded_ = false;
};

} // namespace game
} // namespace wowee
