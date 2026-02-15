#pragma once

#include "game/world_packets.hpp"
#include "game/character.hpp"
#include "game/opcode_table.hpp"
#include "game/update_field_table.hpp"
#include "game/inventory.hpp"
#include "game/spell_defines.hpp"
#include "game/group_defines.hpp"
#include <glm/glm.hpp>
#include <memory>
#include <string>
#include <vector>
#include <deque>
#include <array>
#include <functional>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <map>
#include <optional>
#include <algorithm>

namespace wowee::game {
    class TransportManager;
    class WardenCrypto;
    class WardenMemory;
    class WardenModuleManager;
    class PacketParsers;
}

namespace wowee {
namespace network { class WorldSocket; class Packet; }

namespace game {

struct PlayerSkill {
    uint32_t skillId = 0;
    uint16_t value = 0;
    uint16_t maxValue = 0;
};

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
    // Talent data structures (must be public for use in templates)
    struct TalentEntry {
        uint32_t talentId = 0;
        uint32_t tabId = 0;           // Which talent tree
        uint8_t row = 0;              // Tier (0-10)
        uint8_t column = 0;           // Column (0-3)
        uint32_t rankSpells[5] = {};  // Spell IDs for ranks 1-5
        uint32_t prereqTalent[3] = {}; // Required talents
        uint8_t prereqRank[3] = {};   // Required ranks
        uint8_t maxRank = 0;          // Number of ranks (1-5)
    };

    struct TalentTabEntry {
        uint32_t tabId = 0;
        std::string name;
        uint32_t classMask = 0;       // Which classes can use this tab
        uint8_t orderIndex = 0;       // Display order (0-2)
        std::string backgroundFile;   // Texture path
    };

    GameHandler();
    ~GameHandler();

    /** Access the active opcode table (wire ↔ logical mapping). */
    const OpcodeTable& getOpcodeTable() const { return opcodeTable_; }
    OpcodeTable& getOpcodeTable() { return opcodeTable_; }
    const UpdateFieldTable& getUpdateFieldTable() const { return updateFieldTable_; }
    UpdateFieldTable& getUpdateFieldTable() { return updateFieldTable_; }
    PacketParsers* getPacketParsers() { return packetParsers_.get(); }
    void setPacketParsers(std::unique_ptr<PacketParsers> parsers);

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
                 uint32_t build = 12340,
                 uint32_t realmId = 0);

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
    uint32_t getCurrentMapId() const { return currentMapId_; }
    bool getHomeBind(uint32_t& mapId, glm::vec3& pos) const {
        if (!hasHomeBind_) return false;
        mapId = homeBindMapId_;
        pos = homeBindPos_;
        return true;
    }

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
    void sendTextEmote(uint32_t textEmoteId, uint64_t targetGuid = 0);
    void joinChannel(const std::string& channelName, const std::string& password = "");
    void leaveChannel(const std::string& channelName);
    const std::vector<std::string>& getJoinedChannels() const { return joinedChannels_; }
    std::string getChannelByIndex(int index) const;

    // Chat bubble callback: (senderGuid, message, isYell)
    using ChatBubbleCallback = std::function<void(uint64_t, const std::string&, bool)>;
    void setChatBubbleCallback(ChatBubbleCallback cb) { chatBubbleCallback_ = std::move(cb); }

    // Emote animation callback: (entityGuid, animationId)
    using EmoteAnimCallback = std::function<void(uint64_t, uint32_t)>;
    void setEmoteAnimCallback(EmoteAnimCallback cb) { emoteAnimCallback_ = std::move(cb); }

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
    void unequipToBackpack(EquipSlot equipSlot);

    // Targeting
    void setTarget(uint64_t guid);
    void clearTarget();
    uint64_t getTargetGuid() const { return targetGuid; }
    std::shared_ptr<Entity> getTarget() const;
    bool hasTarget() const { return targetGuid != 0; }
    void tabTarget(float playerX, float playerY, float playerZ);

    // Focus targeting
    void setFocus(uint64_t guid);
    void clearFocus();
    uint64_t getFocusGuid() const { return focusGuid; }
    std::shared_ptr<Entity> getFocus() const;
    bool hasFocus() const { return focusGuid != 0; }

    // Advanced targeting
    void targetLastTarget();
    void targetEnemy(bool reverse = false);
    void targetFriend(bool reverse = false);

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
    void kickGuildMember(const std::string& playerName);
    void acceptGuildInvite();
    void declineGuildInvite();
    void queryGuildInfo(uint32_t guildId);

    // Guild state accessors
    bool isInGuild() const {
        if (!guildName_.empty()) return true;
        const Character* ch = getActiveCharacter();
        return ch && ch->hasGuild();
    }
    const std::string& getGuildName() const { return guildName_; }
    const GuildRosterData& getGuildRoster() const { return guildRoster_; }
    bool hasGuildRoster() const { return hasGuildRoster_; }
    bool hasPendingGuildInvite() const { return pendingGuildInvite_; }
    const std::string& getPendingGuildInviterName() const { return pendingGuildInviterName_; }
    const std::string& getPendingGuildInviteGuildName() const { return pendingGuildInviteGuildName_; }

    // Ready check
    void initiateReadyCheck();
    void respondToReadyCheck(bool ready);

    // Duel
    void forfeitDuel();

    // AFK/DND status
    void toggleAfk(const std::string& message = "");
    void toggleDnd(const std::string& message = "");
    void replyToLastWhisper(const std::string& message);
    std::string getLastWhisperSender() const { return lastWhisperSender_; }
    void setLastWhisperSender(const std::string& name) { lastWhisperSender_ = name; }

    // Party/Raid management
    void uninvitePlayer(const std::string& playerName);
    void leaveParty();
    void setMainTank(uint64_t targetGuid);
    void setMainAssist(uint64_t targetGuid);
    void clearMainTank();
    void clearMainAssist();
    void requestRaidInfo();

    // Combat and Trade
    void proposeDuel(uint64_t targetGuid);
    void initiateTrade(uint64_t targetGuid);
    void stopCasting();

    // ---- Phase 1: Name queries ----
    void queryPlayerName(uint64_t guid);
    void queryCreatureInfo(uint32_t entry, uint64_t guid);
    void queryGameObjectInfo(uint32_t entry, uint64_t guid);
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

    // Talents
    uint8_t getActiveTalentSpec() const { return activeTalentSpec_; }
    uint8_t getUnspentTalentPoints() const { return unspentTalentPoints_[activeTalentSpec_]; }
    uint8_t getUnspentTalentPoints(uint8_t spec) const { return spec < 2 ? unspentTalentPoints_[spec] : 0; }
    const std::unordered_map<uint32_t, uint8_t>& getLearnedTalents() const { return learnedTalents_[activeTalentSpec_]; }
    const std::unordered_map<uint32_t, uint8_t>& getLearnedTalents(uint8_t spec) const {
        static std::unordered_map<uint32_t, uint8_t> empty;
        return spec < 2 ? learnedTalents_[spec] : empty;
    }
    uint8_t getTalentRank(uint32_t talentId) const {
        auto it = learnedTalents_[activeTalentSpec_].find(talentId);
        return (it != learnedTalents_[activeTalentSpec_].end()) ? it->second : 0;
    }
    void learnTalent(uint32_t talentId, uint32_t requestedRank);
    void switchTalentSpec(uint8_t newSpec);

    // Talent DBC access
    const TalentEntry* getTalentEntry(uint32_t talentId) const {
        auto it = talentCache_.find(talentId);
        return (it != talentCache_.end()) ? &it->second : nullptr;
    }
    const TalentTabEntry* getTalentTabEntry(uint32_t tabId) const {
        auto it = talentTabCache_.find(tabId);
        return (it != talentTabCache_.end()) ? &it->second : nullptr;
    }
    const std::unordered_map<uint32_t, TalentEntry>& getAllTalents() const { return talentCache_; }
    const std::unordered_map<uint32_t, TalentTabEntry>& getAllTalentTabs() const { return talentTabCache_; }
    void loadTalentDbc();

    // Action bar
    static constexpr int ACTION_BAR_SLOTS = 12;
    std::array<ActionBarSlot, ACTION_BAR_SLOTS>& getActionBar() { return actionBar; }
    const std::array<ActionBarSlot, ACTION_BAR_SLOTS>& getActionBar() const { return actionBar; }
    void setActionBarSlot(int slot, ActionBarSlot::Type type, uint32_t id);

    void saveCharacterConfig();
    void loadCharacterConfig();
    static std::string getCharacterConfigDir();

    // Auras
    const std::vector<AuraSlot>& getPlayerAuras() const { return playerAuras; }
    const std::vector<AuraSlot>& getTargetAuras() const { return targetAuras; }

    // NPC death callback (for animations)
    using NpcDeathCallback = std::function<void(uint64_t guid)>;
    void setNpcDeathCallback(NpcDeathCallback cb) { npcDeathCallback_ = std::move(cb); }

    using NpcAggroCallback = std::function<void(uint64_t guid, const glm::vec3& position)>;
    void setNpcAggroCallback(NpcAggroCallback cb) { npcAggroCallback_ = std::move(cb); }

    // NPC respawn callback (health 0 → >0, resets animation to idle)
    using NpcRespawnCallback = std::function<void(uint64_t guid)>;
    void setNpcRespawnCallback(NpcRespawnCallback cb) { npcRespawnCallback_ = std::move(cb); }

    // Melee swing callback (for driving animation/SFX)
    using MeleeSwingCallback = std::function<void()>;
    void setMeleeSwingCallback(MeleeSwingCallback cb) { meleeSwingCallback_ = std::move(cb); }

    // NPC swing callback (plays attack animation on NPC)
    using NpcSwingCallback = std::function<void(uint64_t guid)>;
    void setNpcSwingCallback(NpcSwingCallback cb) { npcSwingCallback_ = std::move(cb); }

    // NPC greeting callback (plays voice line when NPC is clicked)
    using NpcGreetingCallback = std::function<void(uint64_t guid, const glm::vec3& position)>;
    void setNpcGreetingCallback(NpcGreetingCallback cb) { npcGreetingCallback_ = std::move(cb); }

    using NpcFarewellCallback = std::function<void(uint64_t guid, const glm::vec3& position)>;
    void setNpcFarewellCallback(NpcFarewellCallback cb) { npcFarewellCallback_ = std::move(cb); }

    using NpcVendorCallback = std::function<void(uint64_t guid, const glm::vec3& position)>;
    void setNpcVendorCallback(NpcVendorCallback cb) { npcVendorCallback_ = std::move(cb); }

    // XP tracking
    uint32_t getPlayerXp() const { return playerXp_; }
    uint32_t getPlayerNextLevelXp() const { return playerNextLevelXp_; }
    uint32_t getPlayerLevel() const { return serverPlayerLevel_; }
    const std::vector<uint32_t>& getPlayerExploredZoneMasks() const { return playerExploredZones_; }
    bool hasPlayerExploredZoneMasks() const { return hasPlayerExploredZones_; }
    static uint32_t killXp(uint32_t playerLevel, uint32_t victimLevel);

    // Server time (for deterministic moon phases, etc.)
    float getGameTime() const { return gameTime_; }
    float getTimeSpeed() const { return timeSpeed_; }

    // Player skills
    const std::map<uint32_t, PlayerSkill>& getPlayerSkills() const { return playerSkills_; }
    const std::string& getSkillName(uint32_t skillId) const;
    uint32_t getSkillCategory(uint32_t skillId) const;

    // World entry callback (online mode - triggered when entering world)
    // Parameters: mapId, x, y, z (canonical WoW coordinates)
    using WorldEntryCallback = std::function<void(uint32_t mapId, float x, float y, float z)>;
    void setWorldEntryCallback(WorldEntryCallback cb) { worldEntryCallback_ = std::move(cb); }

    // Unstuck callback (resets player Z to floor height)
    using UnstuckCallback = std::function<void()>;
    void setUnstuckCallback(UnstuckCallback cb) { unstuckCallback_ = std::move(cb); }
    void unstuck();
    void setUnstuckGyCallback(UnstuckCallback cb) { unstuckGyCallback_ = std::move(cb); }
    void unstuckGy();
    using BindPointCallback = std::function<void(uint32_t mapId, float x, float y, float z)>;
    void setBindPointCallback(BindPointCallback cb) { bindPointCallback_ = std::move(cb); }

    // Creature spawn callback (online mode - triggered when creature enters view)
    // Parameters: guid, displayId, x, y, z (canonical), orientation
    using CreatureSpawnCallback = std::function<void(uint64_t guid, uint32_t displayId, float x, float y, float z, float orientation)>;
    void setCreatureSpawnCallback(CreatureSpawnCallback cb) { creatureSpawnCallback_ = std::move(cb); }

    // Creature despawn callback (online mode - triggered when creature leaves view)
    using CreatureDespawnCallback = std::function<void(uint64_t guid)>;
    void setCreatureDespawnCallback(CreatureDespawnCallback cb) { creatureDespawnCallback_ = std::move(cb); }

    // Player spawn callback (online mode - triggered when a player enters view).
    // Players need appearance data so the renderer can build the right body/hair textures.
    using PlayerSpawnCallback = std::function<void(uint64_t guid,
                                                   uint32_t displayId,
                                                   uint8_t raceId,
                                                   uint8_t genderId,
                                                   uint32_t appearanceBytes,
                                                   uint8_t facialFeatures,
                                                   float x, float y, float z, float orientation)>;
    void setPlayerSpawnCallback(PlayerSpawnCallback cb) { playerSpawnCallback_ = std::move(cb); }

    using PlayerDespawnCallback = std::function<void(uint64_t guid)>;
    void setPlayerDespawnCallback(PlayerDespawnCallback cb) { playerDespawnCallback_ = std::move(cb); }

    // Online player equipment visuals callback.
    // Sends a best-effort view of equipped items for players in view using ItemDisplayInfo IDs.
    // Arrays are indexed by EquipSlot (0..18). Values are 0 when unknown/unavailable.
    using PlayerEquipmentCallback = std::function<void(uint64_t guid,
                                                      const std::array<uint32_t, 19>& displayInfoIds,
                                                      const std::array<uint8_t, 19>& inventoryTypes)>;
    void setPlayerEquipmentCallback(PlayerEquipmentCallback cb) { playerEquipmentCallback_ = std::move(cb); }

    // GameObject spawn callback (online mode - triggered when gameobject enters view)
    // Parameters: guid, entry, displayId, x, y, z (canonical), orientation
    using GameObjectSpawnCallback = std::function<void(uint64_t guid, uint32_t entry, uint32_t displayId, float x, float y, float z, float orientation)>;
    void setGameObjectSpawnCallback(GameObjectSpawnCallback cb) { gameObjectSpawnCallback_ = std::move(cb); }

    // GameObject move callback (online mode - triggered when gameobject position updates)
    // Parameters: guid, x, y, z (canonical), orientation
    using GameObjectMoveCallback = std::function<void(uint64_t guid, float x, float y, float z, float orientation)>;
    void setGameObjectMoveCallback(GameObjectMoveCallback cb) { gameObjectMoveCallback_ = std::move(cb); }

    // GameObject despawn callback (online mode - triggered when gameobject leaves view)
    using GameObjectDespawnCallback = std::function<void(uint64_t guid)>;
    void setGameObjectDespawnCallback(GameObjectDespawnCallback cb) { gameObjectDespawnCallback_ = std::move(cb); }

    // Faction hostility map (populated from FactionTemplate.dbc by Application)
    void setFactionHostileMap(std::unordered_map<uint32_t, bool> map) { factionHostileMap_ = std::move(map); }

    // Creature move callback (online mode - triggered by SMSG_MONSTER_MOVE)
    // Parameters: guid, x, y, z (canonical), duration_ms (0 = instant)
    using CreatureMoveCallback = std::function<void(uint64_t guid, float x, float y, float z, uint32_t durationMs)>;
    void setCreatureMoveCallback(CreatureMoveCallback cb) { creatureMoveCallback_ = std::move(cb); }

    // Transport move callback (online mode - triggered when transport position updates)
    // Parameters: guid, x, y, z (canonical), orientation
    using TransportMoveCallback = std::function<void(uint64_t guid, float x, float y, float z, float orientation)>;
    void setTransportMoveCallback(TransportMoveCallback cb) { transportMoveCallback_ = std::move(cb); }

    // Transport spawn callback (online mode - triggered when transport GameObject is first detected)
    // Parameters: guid, entry, displayId, x, y, z (canonical), orientation
    using TransportSpawnCallback = std::function<void(uint64_t guid, uint32_t entry, uint32_t displayId, float x, float y, float z, float orientation)>;
    void setTransportSpawnCallback(TransportSpawnCallback cb) { transportSpawnCallback_ = std::move(cb); }

    // Notify that a transport has been spawned (called after WMO instance creation)
    void notifyTransportSpawned(uint64_t guid, uint32_t entry, uint32_t displayId, float x, float y, float z, float orientation) {
        if (transportSpawnCallback_) {
            transportSpawnCallback_(guid, entry, displayId, x, y, z, orientation);
        }
    }

    // Transport state for player-on-transport
    bool isOnTransport() const { return playerTransportGuid_ != 0; }
    uint64_t getPlayerTransportGuid() const { return playerTransportGuid_; }
    glm::vec3 getPlayerTransportOffset() const { return playerTransportOffset_; }

    // Check if a GUID is a known transport
    bool isTransportGuid(uint64_t guid) const { return transportGuids_.count(guid) > 0; }
    bool hasServerTransportUpdate(uint64_t guid) const { return serverUpdatedTransportGuids_.count(guid) > 0; }
    glm::vec3 getComposedWorldPosition();  // Compose transport transform * local offset
    TransportManager* getTransportManager() { return transportManager_.get(); }
    void setPlayerOnTransport(uint64_t transportGuid, const glm::vec3& localOffset) {
        playerTransportGuid_ = transportGuid;
        playerTransportOffset_ = localOffset;
        playerTransportStickyGuid_ = transportGuid;
        playerTransportStickyTimer_ = 8.0f;
        movementInfo.transportGuid = transportGuid;
    }
    void clearPlayerTransport() {
        if (playerTransportGuid_ != 0) {
            playerTransportStickyGuid_ = playerTransportGuid_;
            playerTransportStickyTimer_ = std::max(playerTransportStickyTimer_, 1.5f);
        }
        playerTransportGuid_ = 0;
        playerTransportOffset_ = glm::vec3(0.0f);
        movementInfo.transportGuid = 0;
    }

    // Cooldowns
    float getSpellCooldown(uint32_t spellId) const;

    // Player GUID
    uint64_t getPlayerGuid() const { return playerGuid; }
    uint8_t getPlayerClass() const {
        const Character* ch = getActiveCharacter();
        return ch ? static_cast<uint8_t>(ch->characterClass) : 0;
    }
    void setPlayerGuid(uint64_t guid) { playerGuid = guid; }

    // Player death state
    bool isPlayerDead() const { return playerDead_; }
    bool isPlayerGhost() const { return releasedSpirit_; }
    bool showDeathDialog() const { return playerDead_ && !releasedSpirit_; }
    bool showResurrectDialog() const { return resurrectRequestPending_; }
    void releaseSpirit();
    void acceptResurrect();
    void declineResurrect();

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
    void interactWithGameObject(uint64_t guid);
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
        // Objective kill counts: objectiveIndex -> (current, required)
        std::unordered_map<uint32_t, std::pair<uint32_t, uint32_t>> killCounts;
    };
    const std::vector<QuestLogEntry>& getQuestLog() const { return questLog_; }
    void abandonQuest(uint32_t questId);

    // Quest giver status (! and ? markers)
    QuestGiverStatus getQuestGiverStatus(uint64_t guid) const {
        auto it = npcQuestStatus_.find(guid);
        return (it != npcQuestStatus_.end()) ? it->second : QuestGiverStatus::NONE;
    }
    const std::unordered_map<uint64_t, QuestGiverStatus>& getNpcQuestStatuses() const { return npcQuestStatus_; }

    // Mount state
    using MountCallback = std::function<void(uint32_t mountDisplayId)>;  // 0 = dismount
    void setMountCallback(MountCallback cb) { mountCallback_ = std::move(cb); }

    // Taxi terrain precaching callback
    using TaxiPrecacheCallback = std::function<void(const std::vector<glm::vec3>&)>;
    void setTaxiPrecacheCallback(TaxiPrecacheCallback cb) { taxiPrecacheCallback_ = std::move(cb); }

    // Taxi orientation callback (for mount rotation: yaw, pitch, roll in radians)
    using TaxiOrientationCallback = std::function<void(float yaw, float pitch, float roll)>;
    void setTaxiOrientationCallback(TaxiOrientationCallback cb) { taxiOrientationCallback_ = std::move(cb); }

    // Callback for when taxi flight is about to start (after mounting delay, before movement begins)
    using TaxiFlightStartCallback = std::function<void()>;
    void setTaxiFlightStartCallback(TaxiFlightStartCallback cb) { taxiFlightStartCallback_ = std::move(cb); }

    bool isMounted() const { return currentMountDisplayId_ != 0; }
    bool isHostileAttacker(uint64_t guid) const { return hostileAttackers_.count(guid) > 0; }
    float getServerRunSpeed() const { return serverRunSpeed_; }
    void dismount();

    // Taxi / Flight Paths
    bool isTaxiWindowOpen() const { return taxiWindowOpen_; }
    void closeTaxi();
    void activateTaxi(uint32_t destNodeId);
    bool isOnTaxiFlight() const { return onTaxiFlight_; }
    bool isTaxiMountActive() const { return taxiMountActive_; }
    bool isTaxiActivationPending() const { return taxiActivatePending_; }
    void forceClearTaxiAndMovementState();
    const ShowTaxiNodesData& getTaxiData() const { return currentTaxiData_; }
    uint32_t getTaxiCurrentNode() const { return currentTaxiData_.nearestNode; }

    struct TaxiNode {
        uint32_t id = 0;
        uint32_t mapId = 0;
        float x = 0, y = 0, z = 0;
        std::string name;
        uint32_t mountDisplayIdAlliance = 0;
        uint32_t mountDisplayIdHorde = 0;
    };
    struct TaxiPathEdge {
        uint32_t pathId = 0;
        uint32_t fromNode = 0, toNode = 0;
        uint32_t cost = 0;
    };
    struct TaxiPathNode {
        uint32_t id = 0;
        uint32_t pathId = 0;
        uint32_t nodeIndex = 0;
        uint32_t mapId = 0;
        float x = 0, y = 0, z = 0;
    };
    const std::unordered_map<uint32_t, TaxiNode>& getTaxiNodes() const { return taxiNodes_; }
    uint32_t getTaxiCostTo(uint32_t destNodeId) const;

    // Vendor
    void openVendor(uint64_t npcGuid);
    void closeVendor();
    void buyItem(uint64_t vendorGuid, uint32_t itemId, uint32_t slot, uint32_t count);
    void sellItem(uint64_t vendorGuid, uint64_t itemGuid, uint32_t count);
    void sellItemBySlot(int backpackIndex);
    void autoEquipItemBySlot(int backpackIndex);
    void useItemBySlot(int backpackIndex);
    void useItemById(uint32_t itemId);
    bool isVendorWindowOpen() const { return vendorWindowOpen; }
    const ListInventoryData& getVendorItems() const { return currentVendorItems; }

    // Trainer
    bool isTrainerWindowOpen() const { return trainerWindowOpen_; }
    const TrainerListData& getTrainerSpells() const { return currentTrainerList_; }
    void trainSpell(uint32_t spellId);
    void closeTrainer();
    const std::string& getSpellName(uint32_t spellId) const;
    const std::string& getSpellRank(uint32_t spellId) const;
    const std::string& getSkillLineName(uint32_t spellId) const;

    struct TrainerTab {
        std::string name;
        std::vector<const TrainerSpell*> spells;
    };
    const std::vector<TrainerTab>& getTrainerTabs() const { return trainerTabs_; }
    const ItemQueryResponseData* getItemInfo(uint32_t itemId) const {
        auto it = itemInfoCache_.find(itemId);
        return (it != itemInfoCache_.end()) ? &it->second : nullptr;
    }
    // Request item info from server if not already cached/pending
    void ensureItemInfo(uint32_t entry) {
        if (entry == 0 || itemInfoCache_.count(entry) || pendingItemQueries_.count(entry)) return;
        queryItemInfo(entry, 0);
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
    void autoTargetAttacker(uint64_t attackerGuid);

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
     * Handle SMSG_CLIENTCACHE_VERSION from server
     */
    void handleClientCacheVersion(network::Packet& packet);

    /**
     * Handle SMSG_TUTORIAL_FLAGS from server
     */
    void handleTutorialFlags(network::Packet& packet);

    /**
     * Handle SMSG_WARDEN_DATA gate packet from server.
     * We do not implement anti-cheat exchange for third-party realms.
     */
    void handleWardenData(network::Packet& packet);

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
    void handleTextEmote(network::Packet& packet);
    void handleChannelNotify(network::Packet& packet);
    void autoJoinDefaultChannels();

    // ---- Phase 1 handlers ----
    void handleNameQueryResponse(network::Packet& packet);
    void handleCreatureQueryResponse(network::Packet& packet);
    void handleGameObjectQueryResponse(network::Packet& packet);
    void handleItemQueryResponse(network::Packet& packet);
    void handleInspectResults(network::Packet& packet);
    void queryItemInfo(uint32_t entry, uint64_t guid);
    void rebuildOnlineInventory();
    void maybeDetectVisibleItemLayout();
    void updateOtherPlayerVisibleItems(uint64_t guid, const std::map<uint16_t, uint32_t>& fields);
    void emitOtherPlayerEquipment(uint64_t guid);
    void emitAllOtherPlayerEquipment();
    void detectInventorySlotBases(const std::map<uint16_t, uint32_t>& fields);
    bool applyInventoryFields(const std::map<uint16_t, uint32_t>& fields);
    void extractContainerFields(uint64_t containerGuid, const std::map<uint16_t, uint32_t>& fields);
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
    void handleSupercededSpell(network::Packet& packet);
    void handleRemovedSpell(network::Packet& packet);
    void handleUnlearnSpells(network::Packet& packet);

    // ---- Talent handlers ----
    void handleTalentsInfo(network::Packet& packet);

    // ---- Phase 4 handlers ----
    void handleGroupInvite(network::Packet& packet);
    void handleGroupDecline(network::Packet& packet);
    void handleGroupList(network::Packet& packet);
    void handleGroupUninvite(network::Packet& packet);
    void handlePartyCommandResult(network::Packet& packet);

    // ---- Guild handlers ----
    void handleGuildInfo(network::Packet& packet);
    void handleGuildRoster(network::Packet& packet);
    void handleGuildQueryResponse(network::Packet& packet);
    void handleGuildEvent(network::Packet& packet);
    void handleGuildInvite(network::Packet& packet);
    void handleGuildCommandResult(network::Packet& packet);

    // ---- Character creation handler ----
    void handleCharCreateResponse(network::Packet& packet);

    // ---- XP handler ----
    void handleXpGain(network::Packet& packet);

    // ---- Creature movement handler ----
    void handleMonsterMove(network::Packet& packet);
    void handleMonsterMoveTransport(network::Packet& packet);

    // ---- Other player movement (MSG_MOVE_* from server) ----
    void handleOtherPlayerMovement(network::Packet& packet);

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

    // ---- Teleport handler ----
    void handleTeleportAck(network::Packet& packet);
    void handleNewWorld(network::Packet& packet);

    // ---- Speed change handler ----
    void handleForceRunSpeedChange(network::Packet& packet);

    // ---- Arena / Battleground handlers ----
    void handleBattlefieldStatus(network::Packet& packet);
    void handleArenaTeamCommandResult(network::Packet& packet);
    void handleArenaTeamQueryResponse(network::Packet& packet);
    void handleArenaTeamInvite(network::Packet& packet);
    void handleArenaTeamEvent(network::Packet& packet);
    void handleArenaError(network::Packet& packet);

    // ---- Taxi handlers ----
    void handleShowTaxiNodes(network::Packet& packet);
    void handleActivateTaxiReply(network::Packet& packet);
    void loadTaxiDbc();

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
    void updateAttachedTransportChildren(float deltaTime);
    void setTransportAttachment(uint64_t childGuid, ObjectType type, uint64_t transportGuid,
                                const glm::vec3& localOffset, bool hasLocalOrientation,
                                float localOrientation);
    void clearTransportAttachment(uint64_t childGuid);

    // Opcode translation table (expansion-specific wire ↔ logical mapping)
    OpcodeTable opcodeTable_;

    // Update field table (expansion-specific field index mapping)
    UpdateFieldTable updateFieldTable_;

    // Packet parsers (expansion-specific binary format handling)
    std::unique_ptr<PacketParsers> packetParsers_;

    // Network
    std::unique_ptr<network::WorldSocket> socket;

    // State
    WorldState state = WorldState::DISCONNECTED;

    // Authentication data
    std::vector<uint8_t> sessionKey;    // 40-byte session key from auth server
    std::string accountName;             // Account name
    uint32_t build = 12340;              // Client build (3.3.5a)
    uint32_t realmId_ = 0;               // Realm ID from auth REALM_LIST (used in WotLK AUTH_SESSION)
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
    std::vector<std::string> joinedChannels_;   // Active channel memberships
    ChatBubbleCallback chatBubbleCallback_;
    EmoteAnimCallback emoteAnimCallback_;

    // Targeting
    uint64_t targetGuid = 0;
    uint64_t focusGuid = 0;              // Focus target
    uint64_t lastTargetGuid = 0;         // Previous target
    std::vector<uint64_t> tabCycleList;
    int tabCycleIndex = -1;
    bool tabCycleStale = true;

    // Heartbeat
    uint32_t pingSequence = 0;               // Ping sequence number (increments)
    float timeSinceLastPing = 0.0f;          // Time since last ping sent (seconds)
    float pingInterval = 30.0f;              // Ping interval (30 seconds)
    float timeSinceLastMoveHeartbeat_ = 0.0f; // Periodic movement heartbeat to keep server position synced
    float moveHeartbeatInterval_ = 0.5f;
    uint32_t lastLatency = 0;                // Last measured latency (milliseconds)

    // Player GUID and map
    uint64_t playerGuid = 0;
    uint32_t currentMapId_ = 0;
    bool hasHomeBind_ = false;
    uint32_t homeBindMapId_ = 0;
    glm::vec3 homeBindPos_{0.0f};

    // ---- Phase 1: Name caches ----
    std::unordered_map<uint64_t, std::string> playerNameCache;
    std::unordered_set<uint64_t> pendingNameQueries;
    std::unordered_map<uint32_t, CreatureQueryResponseData> creatureInfoCache;
    std::unordered_set<uint32_t> pendingCreatureQueries;
    std::unordered_map<uint32_t, GameObjectQueryResponseData> gameObjectInfoCache_;
    std::unordered_set<uint32_t> pendingGameObjectQueries_;

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

    // ---- AFK/DND status ----
    bool afkStatus_ = false;
    bool dndStatus_ = false;
    std::string afkMessage_;
    std::string dndMessage_;
    std::string lastWhisperSender_;

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
    // Container (bag) contents: containerGuid -> array of item GUIDs per slot
    struct ContainerInfo {
        uint32_t numSlots = 0;
        std::array<uint64_t, 36> slotGuids{};  // max 36 slots
    };
    std::unordered_map<uint64_t, ContainerInfo> containerContents_;
    int invSlotBase_ = -1;
    int packSlotBase_ = -1;
    std::map<uint16_t, uint32_t> lastPlayerFields_;
    bool onlineEquipDirty_ = false;
    std::array<uint32_t, 19> lastEquipDisplayIds_{};

    // Visible equipment for other players: detect the update-field layout (base + stride)
    // using the local player's own equipped items, then decode other players by index.
    // Default to known WotLK 3.3.5a layout: UNIT_END(148) + 0x0088 = 284, stride 2.
    // The heuristic in maybeDetectVisibleItemLayout() can still override if needed.
    int visibleItemEntryBase_ = 284;
    int visibleItemStride_ = 2;
    bool visibleItemLayoutVerified_ = false;  // true once heuristic confirms/overrides default
    std::unordered_map<uint64_t, std::array<uint32_t, 19>> otherPlayerVisibleItemEntries_;
    std::unordered_set<uint64_t> otherPlayerVisibleDirty_;
    std::unordered_map<uint64_t, uint32_t> otherPlayerMoveTimeMs_;

    // Inspect fallback (when visible item fields are missing/unreliable)
    std::unordered_map<uint64_t, std::array<uint32_t, 19>> inspectedPlayerItemEntries_;
    std::unordered_set<uint64_t> pendingAutoInspect_;
    float inspectRateLimit_ = 0.0f;

    // ---- Phase 2: Combat ----
    bool autoAttacking = false;
    uint64_t autoAttackTarget = 0;
    bool autoAttackOutOfRange_ = false;
    std::unordered_set<uint64_t> hostileAttackers_;
    std::vector<CombatTextEntry> combatText;

    // ---- Phase 3: Spells ----
    WorldEntryCallback worldEntryCallback_;
    UnstuckCallback unstuckCallback_;
    UnstuckCallback unstuckGyCallback_;
    BindPointCallback bindPointCallback_;
    CreatureSpawnCallback creatureSpawnCallback_;
    CreatureDespawnCallback creatureDespawnCallback_;
    PlayerSpawnCallback playerSpawnCallback_;
    PlayerDespawnCallback playerDespawnCallback_;
    PlayerEquipmentCallback playerEquipmentCallback_;
    CreatureMoveCallback creatureMoveCallback_;
    TransportMoveCallback transportMoveCallback_;
    TransportSpawnCallback transportSpawnCallback_;
    GameObjectSpawnCallback gameObjectSpawnCallback_;
    GameObjectMoveCallback gameObjectMoveCallback_;
    GameObjectDespawnCallback gameObjectDespawnCallback_;

    // Transport tracking
    struct TransportAttachment {
        ObjectType type = ObjectType::OBJECT;
        uint64_t transportGuid = 0;
        glm::vec3 localOffset{0.0f};
        float localOrientation = 0.0f;
        bool hasLocalOrientation = false;
    };
    std::unordered_map<uint64_t, TransportAttachment> transportAttachments_;
    std::unordered_set<uint64_t> transportGuids_;  // GUIDs of known transport GameObjects
    std::unordered_set<uint64_t> serverUpdatedTransportGuids_;
    uint64_t playerTransportGuid_ = 0;             // Transport the player is riding (0 = none)
    glm::vec3 playerTransportOffset_ = glm::vec3(0.0f); // Player offset on transport
    uint64_t playerTransportStickyGuid_ = 0;       // Last transport player was on (temporary retention)
    float playerTransportStickyTimer_ = 0.0f;      // Seconds to keep sticky transport alive after transient clears
    std::unique_ptr<TransportManager> transportManager_;  // Transport movement manager
    std::vector<uint32_t> knownSpells;
    std::unordered_map<uint32_t, float> spellCooldowns;    // spellId -> remaining seconds
    uint8_t castCount = 0;
    bool casting = false;
    uint32_t currentCastSpellId = 0;
    float castTimeRemaining = 0.0f;

    // Talents (dual-spec support)
    uint8_t activeTalentSpec_ = 0;                              // Currently active spec (0 or 1)
    uint8_t unspentTalentPoints_[2] = {0, 0};                   // Unspent points per spec
    std::unordered_map<uint32_t, uint8_t> learnedTalents_[2];  // Learned talents per spec
    std::unordered_map<uint32_t, TalentEntry> talentCache_;      // talentId -> entry
    std::unordered_map<uint32_t, TalentTabEntry> talentTabCache_; // tabId -> entry
    bool talentDbcLoaded_ = false;
    float castTimeTotal = 0.0f;
    std::array<ActionBarSlot, 12> actionBar{};
    std::vector<AuraSlot> playerAuras;
    std::vector<AuraSlot> targetAuras;

    // ---- Phase 4: Group ----
    GroupListData partyData;
    bool pendingGroupInvite = false;
    std::string pendingInviterName;

    // ---- Guild state ----
    std::string guildName_;
    std::vector<std::string> guildRankNames_;
    GuildRosterData guildRoster_;
    bool hasGuildRoster_ = false;
    bool pendingGuildInvite_ = false;
    std::string pendingGuildInviterName_;
    std::string pendingGuildInviteGuildName_;

    uint64_t activeCharacterGuid_ = 0;
    Race playerRace_ = Race::HUMAN;

    // ---- Phase 5: Loot ----
    bool lootWindowOpen = false;
    LootResponseData currentLoot;
    struct LocalLootState {
        LootResponseData data;
        bool moneyTaken = false;
    };
    std::unordered_map<uint64_t, LocalLootState> localLootState_;
    uint64_t playerMoneyCopper_ = 0;
    // Some servers/custom clients shift update field indices. We can auto-detect coinage by correlating
    // money-notify deltas with update-field diffs and then overriding UF::PLAYER_FIELD_COINAGE at runtime.
    uint32_t pendingMoneyDelta_ = 0;
    float pendingMoneyDeltaTimer_ = 0.0f;

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

    // Taxi / Flight Paths
    std::unordered_map<uint32_t, TaxiNode> taxiNodes_;
    std::vector<TaxiPathEdge> taxiPathEdges_;
    std::unordered_map<uint32_t, std::vector<TaxiPathNode>> taxiPathNodes_;  // pathId -> ordered waypoints
    bool taxiDbcLoaded_ = false;
    bool taxiWindowOpen_ = false;
    ShowTaxiNodesData currentTaxiData_;
    uint64_t taxiNpcGuid_ = 0;
    bool onTaxiFlight_ = false;
    bool taxiMountActive_ = false;
    uint32_t taxiMountDisplayId_ = 0;
    bool taxiActivatePending_ = false;
    float taxiActivateTimer_ = 0.0f;
    bool taxiClientActive_ = false;
    float taxiLandingCooldown_ = 0.0f;  // Prevent re-entering taxi right after landing
    float taxiStartGrace_ = 0.0f;       // Ignore transient landing/dismount checks right after takeoff
    size_t taxiClientIndex_ = 0;
    std::vector<glm::vec3> taxiClientPath_;
    float taxiClientSpeed_ = 32.0f;
    float taxiClientSegmentProgress_ = 0.0f;
    bool taxiRecoverPending_ = false;
    uint32_t taxiRecoverMapId_ = 0;
    glm::vec3 taxiRecoverPos_{0.0f};
    uint32_t knownTaxiMask_[12] = {};  // Track previously known nodes for discovery alerts
    bool taxiMaskInitialized_ = false; // First SMSG_SHOWTAXINODES seeds mask without alerts
    std::unordered_map<uint32_t, uint32_t> taxiCostMap_; // destNodeId -> total cost in copper
    void buildTaxiCostMap();
    void applyTaxiMountForCurrentNode();
    void sanitizeMovementForTaxi();
    void startClientTaxiPath(const std::vector<uint32_t>& pathNodes);
    void updateClientTaxi(float deltaTime);

    // Vendor
    bool vendorWindowOpen = false;
    ListInventoryData currentVendorItems;

    // Trainer
    bool trainerWindowOpen_ = false;
    TrainerListData currentTrainerList_;
    struct SpellNameEntry { std::string name; std::string rank; };
    std::unordered_map<uint32_t, SpellNameEntry> spellNameCache_;
    bool spellNameCacheLoaded_ = false;
    std::vector<TrainerTab> trainerTabs_;
    void handleTrainerList(network::Packet& packet);
    void loadSpellNameCache();
    void categorizeTrainerSpells();

    // Callbacks
    WorldConnectSuccessCallback onSuccess;
    WorldConnectFailureCallback onFailure;
    CharCreateCallback charCreateCallback_;
    CharDeleteCallback charDeleteCallback_;
    uint8_t lastCharDeleteResult_ = 0xFF;
    bool pendingCharCreateResult_ = false;
    bool pendingCharCreateSuccess_ = false;
    std::string pendingCharCreateMsg_;
    bool requiresWarden_ = false;
    bool wardenGateSeen_ = false;
    float wardenGateElapsed_ = 0.0f;
    float wardenGateNextStatusLog_ = 2.0f;
    uint32_t wardenPacketsAfterGate_ = 0;
    bool wardenCharEnumBlockedLogged_ = false;
    std::unique_ptr<WardenCrypto> wardenCrypto_;
    std::unique_ptr<WardenMemory> wardenMemory_;
    std::unique_ptr<WardenModuleManager> wardenModuleManager_;

    // Warden module download state
    enum class WardenState {
        WAIT_MODULE_USE,     // Waiting for first SMSG (MODULE_USE)
        WAIT_MODULE_CACHE,   // Sent MODULE_MISSING, receiving module chunks
        WAIT_HASH_REQUEST,   // Module received, waiting for HASH_REQUEST
        WAIT_CHECKS,         // Hash sent, waiting for check requests
    };
    WardenState wardenState_ = WardenState::WAIT_MODULE_USE;
    std::vector<uint8_t> wardenModuleHash_;    // 16 bytes MD5
    std::vector<uint8_t> wardenModuleKey_;     // 16 bytes RC4
    uint32_t wardenModuleSize_ = 0;
    std::vector<uint8_t> wardenModuleData_;    // Downloaded module chunks
    std::vector<uint8_t> wardenLoadedModuleImage_; // Parsed module image for key derivation

    // Pre-computed challenge/response entries from .cr file
    struct WardenCREntry {
        uint8_t seed[16];
        uint8_t reply[20];
        uint8_t clientKey[16];  // Encrypt key (client→server)
        uint8_t serverKey[16]; // Decrypt key (server→client)
    };
    std::vector<WardenCREntry> wardenCREntries_;
    // Module-specific check type opcodes [9]: MEM, PAGE_A, PAGE_B, MPQ, LUA, DRIVER, TIMING, PROC, MODULE
    uint8_t wardenCheckOpcodes_[9] = {};
    bool loadWardenCRFile(const std::string& moduleHashHex);

    // ---- XP tracking ----
    uint32_t playerXp_ = 0;
    uint32_t playerNextLevelXp_ = 0;
    uint32_t serverPlayerLevel_ = 1;
    static uint32_t xpForLevel(uint32_t level);

    // ---- Server time tracking (for deterministic celestial/sky systems) ----
    float gameTime_ = 0.0f;       // Server game time in seconds
    float timeSpeed_ = 0.0166f;   // Time scale (default: 1 game day = 1 real hour)
    void handleLoginSetTimeSpeed(network::Packet& packet);

    // ---- Player skills ----
    std::map<uint32_t, PlayerSkill> playerSkills_;
    std::unordered_map<uint32_t, std::string> skillLineNames_;
    std::unordered_map<uint32_t, uint32_t> skillLineCategories_;
    std::unordered_map<uint32_t, uint32_t> spellToSkillLine_;      // spellID -> skillLineID
    bool skillLineDbcLoaded_ = false;
    bool skillLineAbilityLoaded_ = false;
    static constexpr size_t PLAYER_EXPLORED_ZONES_COUNT = 128;
    std::vector<uint32_t> playerExploredZones_ =
        std::vector<uint32_t>(PLAYER_EXPLORED_ZONES_COUNT, 0u);
    bool hasPlayerExploredZones_ = false;
    void loadSkillLineDbc();
    void loadSkillLineAbilityDbc();
    void extractSkillFields(const std::map<uint16_t, uint32_t>& fields);
    void extractExploredZoneFields(const std::map<uint16_t, uint32_t>& fields);

    NpcDeathCallback npcDeathCallback_;
    NpcAggroCallback npcAggroCallback_;
    NpcRespawnCallback npcRespawnCallback_;
    MeleeSwingCallback meleeSwingCallback_;
    NpcSwingCallback npcSwingCallback_;
    NpcGreetingCallback npcGreetingCallback_;
    NpcFarewellCallback npcFarewellCallback_;
    NpcVendorCallback npcVendorCallback_;
    MountCallback mountCallback_;
    TaxiPrecacheCallback taxiPrecacheCallback_;
    TaxiOrientationCallback taxiOrientationCallback_;
    TaxiFlightStartCallback taxiFlightStartCallback_;
    uint32_t currentMountDisplayId_ = 0;
    uint32_t mountAuraSpellId_ = 0;       // Spell ID of the aura that caused mounting (for CMSG_CANCEL_AURA fallback)
    float serverRunSpeed_ = 7.0f;
    bool playerDead_ = false;
    bool releasedSpirit_ = false;
    uint64_t pendingSpiritHealerGuid_ = 0;
    bool resurrectPending_ = false;
    bool resurrectRequestPending_ = false;
    uint64_t resurrectCasterGuid_ = 0;
    bool repopPending_ = false;
    uint64_t lastRepopRequestMs_ = 0;
};

} // namespace game
} // namespace wowee
