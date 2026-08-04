#pragma once

#include "game/game_interfaces.hpp"
#include "game/world_packets.hpp"
#include "game/character.hpp"
#include "game/opcode_table.hpp"
#include "game/update_field_table.hpp"
#include "game/inventory.hpp"
#include "game/spell_defines.hpp"
#include "game/group_defines.hpp"
#include "game/handler_types.hpp"
#include "game/combat_handler.hpp"
#include "game/spell_handler.hpp"
#include "game/quest_handler.hpp"
#include "game/movement_handler.hpp"
#include "game/entity_controller.hpp"
#include "game/game_services.hpp"
#include "network/packet.hpp"
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
#include <chrono>
#include <future>

namespace wowee::game {
    class TransportManager;
    class WardenCrypto;
    class WardenMemory;
    class WardenModule;
    class WardenModuleManager;
    class PacketParsers;
    class ChatHandler;
    class MovementHandler;
    class InventoryHandler;
    class SocialHandler;
    class WardenHandler;
}

namespace wowee {
namespace network { class WorldSocket; class Packet; }
namespace audio { enum class PlayerErrorSpeech : uint8_t; }

namespace game {

struct PlayerSkill {
    uint32_t skillId = 0;
    uint16_t value = 0;        // base + permanent item bonuses
    uint16_t maxValue = 0;
    uint16_t bonusTemp = 0;    // temporary buff bonus (food, potions, etc.)
    uint16_t bonusPerm = 0;    // permanent spec/misc bonus (rarely non-zero)
    uint16_t effectiveValue() const { return value + bonusTemp + bonusPerm; }
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
 * A single contact list entry (friend, ignore, or mute).
 */
struct ContactEntry {
    uint64_t    guid     = 0;
    std::string name;
    std::string note;
    uint32_t    flags    = 0;   // 0x1=friend, 0x2=ignore, 0x4=mute
    uint8_t     status   = 0;   // 0=offline, 1=online, 2=AFK, 3=DND
    uint32_t    areaId   = 0;
    uint32_t    level    = 0;
    uint32_t    classId  = 0;

    bool isFriend() const { return (flags & 0x1) != 0; }
    bool isIgnored() const { return (flags & 0x2) != 0; }
    bool isOnline()  const { return status != 0; }
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
class GameHandler : public IConnectionState,
                     public ITargetingState,
                     public IEntityAccess,
                     public ISocialState,
                     public IPvpState {
public:
    // Talent data structures (aliased from handler_types.hpp)
    using TalentEntry = game::TalentEntry;
    using TalentTabEntry = game::TalentTabEntry;

    explicit GameHandler(GameServices& services);
    ~GameHandler();

    const GameServices& services() const { return services_; }

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
    bool isInWorld() const { return state == WorldState::IN_WORLD && socket; }

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

    using CharDeleteCallback = std::function<void(bool success, const std::string& message)>;
    void setCharDeleteCallback(CharDeleteCallback cb) { charDeleteCallback_ = std::move(cb); }
    uint8_t getLastCharDeleteResult() const { return lastCharDeleteResult_; }

    using CharLoginFailCallback = std::function<void(const std::string& reason)>;
    void setCharLoginFailCallback(CharLoginFailCallback cb) { charLoginFailCallback_ = std::move(cb); }

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
    uint32_t getHomeBindZoneId() const { return homeBindZoneId_; }

    /**
     * Send a movement packet
     * @param opcode Movement opcode (MSG_MOVE_START_FORWARD, etc.)
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
    EntityManager& getEntityManager() { return entityController_->getEntityManager(); }
    const EntityManager& getEntityManager() const { return entityController_->getEntityManager(); }

    /**
     * Send a chat message
     * @param type Chat type (SAY, YELL, WHISPER, etc.)
     * @param message Message text
     * @param target Target name (for whispers, empty otherwise)
     */
    void sendChatMessage(ChatType type, const std::string& message, const std::string& target = "");
    void sendAddonMessage(ChatType type, const std::string& message, const std::string& target = "");
    void sendTextEmote(uint32_t textEmoteId, uint64_t targetGuid = 0);
    void joinChannel(const std::string& channelName, const std::string& password = "");
    void leaveChannel(const std::string& channelName);
    const std::vector<std::string>& getJoinedChannels() const;
    std::string getChannelByIndex(int index) const;
    int getChannelIndex(const std::string& channelName) const;

    // Chat auto-join settings (aliased from handler_types.hpp)
    using ChatAutoJoin = game::ChatAutoJoin;
    ChatAutoJoin chatAutoJoin;
    void autoJoinDefaultChannels();

    // Chat bubble callback: (senderGuid, message, isYell)
    using ChatBubbleCallback = std::function<void(uint64_t, const std::string&, bool)>;
    void setChatBubbleCallback(ChatBubbleCallback cb) { chatBubbleCallback_ = std::move(cb); }

    // Addon chat event callback: fires when any chat message is received (for Lua event dispatch)
    using AddonChatCallback = std::function<void(const MessageChatData&)>;
    void setAddonChatCallback(AddonChatCallback cb) { addonChatCallback_ = std::move(cb); }

    // Generic addon event callback: fires named events with string args
    using AddonEventCallback = std::function<void(const std::string&, const std::vector<std::string>&)>;
    void setAddonEventCallback(AddonEventCallback cb) { addonEventCallback_ = std::move(cb); }

    // Spell icon path resolver: spellId -> texture path string (e.g., "Interface\\Icons\\Spell_Fire_Fireball01")
    using SpellIconPathResolver = std::function<std::string(uint32_t)>;
    void setSpellIconPathResolver(SpellIconPathResolver r) { spellIconPathResolver_ = std::move(r); }
    std::string getSpellIconPath(uint32_t spellId) const {
        return spellIconPathResolver_ ? spellIconPathResolver_(spellId) : std::string{};
    }

    // Spell data resolver: spellId -> {castTimeMs, minRange, maxRange}
    struct SpellDataInfo { uint32_t castTimeMs = 0; float minRange = 0; float maxRange = 0; uint32_t manaCost = 0; uint8_t powerType = 0; };
    using SpellDataResolver = std::function<SpellDataInfo(uint32_t)>;
    void setSpellDataResolver(SpellDataResolver r) { spellDataResolver_ = std::move(r); }
    SpellDataInfo getSpellData(uint32_t spellId) const {
        return spellDataResolver_ ? spellDataResolver_(spellId) : SpellDataInfo{};
    }

    // Item icon path resolver: displayInfoId -> texture path (e.g., "Interface\\Icons\\INV_Sword_04")
    using ItemIconPathResolver = std::function<std::string(uint32_t)>;
    void setItemIconPathResolver(ItemIconPathResolver r) { itemIconPathResolver_ = std::move(r); }
    std::string getItemIconPath(uint32_t displayInfoId) const {
        return itemIconPathResolver_ ? itemIconPathResolver_(displayInfoId) : std::string{};
    }

    // Random property/suffix name resolver: randomPropertyId -> suffix name (e.g., "of the Eagle")
    // Positive IDs → ItemRandomProperties.dbc; negative IDs → ItemRandomSuffix.dbc (abs value)
    using RandomPropertyNameResolver = std::function<std::string(int32_t)>;
    void setRandomPropertyNameResolver(RandomPropertyNameResolver r) { randomPropertyNameResolver_ = std::move(r); }
    std::string getRandomPropertyName(int32_t id) const {
        return randomPropertyNameResolver_ ? randomPropertyNameResolver_(id) : std::string{};
    }

    // Random-suffix/property stat bonuses. Given the item's signed randomPropertyId and its
    // suffixFactor (ITEM_FIELD_PROPERTY_SEED), returns the rolled {statType, value} pairs a
    // green/blue item gains from "of the Bear" etc. statType uses the ITEM_MOD codes shared
    // with ItemDef::ExtraStat (4=Str, 7=Sta, 45=SpellPower, ...). Suffix stats scale as
    // AllocationPct*suffixFactor/10000; positive properties use the enchant's fixed amount.
    struct RandomStatBonus { uint32_t statType = 0; int32_t value = 0; };
    using RandomStatResolver = std::function<std::vector<RandomStatBonus>(int32_t, uint32_t)>;
    void setRandomStatResolver(RandomStatResolver r) { randomStatResolver_ = std::move(r); }
    std::vector<RandomStatBonus> getRandomStatBonuses(int32_t id, uint32_t suffixFactor) const {
        return randomStatResolver_ ? randomStatResolver_(id, suffixFactor)
                                   : std::vector<RandomStatBonus>{};
    }

    // Emote animation callback: (entityGuid, animationId)
    // guid, animId, isState. isState marks persistent STATE_ emotes (from
    // UNIT_NPC_EMOTESTATE or state-type SMSG_EMOTE) that loop until cleared;
    // one-shots play once and return to the prior state.
    using EmoteAnimCallback = std::function<void(uint64_t, uint32_t, bool)>;
    void setEmoteAnimCallback(EmoteAnimCallback cb) { emoteAnimCallback_ = std::move(cb); }

    /**
     * Get chat history (recent messages)
     * @param maxMessages Maximum number of messages to return (0 = all)
     * @return Vector of chat messages
     */
    const std::deque<MessageChatData>& getChatHistory() const;
    void clearChatHistory();

    /**
     * Add a locally-generated chat message (e.g., emote feedback)
     */
    void addLocalChatMessage(const MessageChatData& msg);

    // Money (copper)
    uint64_t getMoneyCopper() const { return playerMoneyCopper_; }

    // PvP currency (TBC/WotLK only)
    uint32_t getHonorPoints() const { return playerHonorPoints_; }
    uint32_t getArenaPoints() const { return playerArenaPoints_; }

    // Server-authoritative armor (UNIT_FIELD_RESISTANCES[0])
    int32_t getArmorRating() const { return playerArmorRating_; }

    // Server-authoritative elemental resistances (UNIT_FIELD_RESISTANCES[1-6]).
    // school: 1=Holy, 2=Fire, 3=Nature, 4=Frost, 5=Shadow, 6=Arcane. Returns 0 if not received.
    int32_t getResistance(int school) const {
        if (school < 1 || school > 6) return 0;
        return playerResistances_[school - 1];
    }

    // Server-authoritative primary stats (UNIT_FIELD_STAT0-4: STR, AGI, STA, INT, SPI).
    // Returns -1 if the server hasn't sent the value yet.
    int32_t getPlayerStat(int idx) const {
        if (idx < 0 || idx > 4) return -1;
        return playerStats_[idx];
    }

    // Server-authoritative attack power (WotLK: UNIT_FIELD_ATTACK_POWER / RANGED).
    // Returns -1 if not yet received.
    int32_t getMeleeAttackPower()  const { return playerMeleeAP_; }
    int32_t getRangedAttackPower() const { return playerRangedAP_; }

    // Server-authoritative spell damage / healing bonus (WotLK: PLAYER_FIELD_MOD_*).
    // getSpellPower returns the max damage bonus across magic schools 1-6 (Holy/Fire/Nature/Frost/Shadow/Arcane).
    // Returns -1 if not yet received.
    int32_t getSpellPower() const {
        int32_t sp = -1;
        for (int i = 1; i <= 6; ++i) {
            if (playerSpellDmgBonus_[i] > sp) sp = playerSpellDmgBonus_[i];
        }
        return sp;
    }
    int32_t getHealingPower() const { return playerHealBonus_; }

    // Server-authoritative combat chance percentages (WotLK: PLAYER_* float fields).
    // Returns -1.0f if not yet received.
    float getDodgePct()  const { return playerDodgePct_; }
    float getParryPct()  const { return playerParryPct_; }
    float getBlockPct()  const { return playerBlockPct_; }
    float getCritPct()   const { return playerCritPct_; }
    float getRangedCritPct() const { return playerRangedCritPct_; }
    // Spell crit by school (0=Physical,1=Holy,2=Fire,3=Nature,4=Frost,5=Shadow,6=Arcane)
    float getSpellCritPct(int school = 1) const {
        if (school < 0 || school > 6) return -1.0f;
        return playerSpellCritPct_[school];
    }

    // Server-authoritative combat ratings (WotLK: PLAYER_FIELD_COMBAT_RATING_1+idx).
    // Returns -1 if not yet received. Indices match AzerothCore CombatRating enum.
    int32_t getCombatRating(int cr) const {
        if (cr < 0 || cr > 24) return -1;
        return playerCombatRatings_[cr];
    }

    // Inventory
    Inventory& getInventory() { return inventory; }
    const Inventory& getInventory() const { return inventory; }
    bool consumeOnlineEquipmentDirty() { bool d = onlineEquipDirty_; onlineEquipDirty_ = false; return d; }
    /// Ask for the equipment visuals to be rebuilt — the helm toggle changes what
    /// is drawn without any item moving.
    void markOnlineEquipmentDirty() { onlineEquipDirty_ = true; }
    void resetEquipmentDirtyTracking() { lastEquipDisplayIds_ = {}; onlineEquipDirty_ = true; }
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

    // Mouseover targeting — set each frame by the nameplate renderer
    void setMouseoverGuid(uint64_t guid);
    uint64_t getMouseoverGuid() const { return mouseoverGuid_; }

    // Advanced targeting
    void targetLastTarget();
    void targetEnemy(bool reverse = false);
    void targetFriend(bool reverse = false);

    // Inspection
    void inspectTarget();

    using InspectArenaTeam = game::InspectArenaTeam;
    using InspectResult = game::InspectResult;
    const InspectResult* getInspectResult() const;
    const std::array<uint32_t, 19>* getOtherPlayerVisibleEquipment(uint64_t guid) const {
        auto it = otherPlayerVisibleItemEntries_.find(guid);
        return (it != otherPlayerVisibleItemEntries_.end()) ? &it->second : nullptr;
    }

    // Server info commands
    void queryServerTime();
    void requestPlayedTime();
    void queryWho(const std::string& playerName = "");
    uint32_t getTotalTimePlayed() const;
    uint32_t getLevelTimePlayed() const;

    using WhoEntry = game::WhoEntry;
    const std::vector<WhoEntry>& getWhoResults() const;
    uint32_t getWhoOnlineCount() const;
    std::string getWhoAreaName(uint32_t zoneId) const { return getAreaName(zoneId); }

    // Social commands
    void addFriend(const std::string& playerName, const std::string& note = "");
    void removeFriend(const std::string& playerName);
    void setFriendNote(const std::string& playerName, const std::string& note);
    void addIgnore(const std::string& playerName);
    void removeIgnore(const std::string& playerName);
    const std::unordered_map<std::string, uint64_t>& getIgnoreCache() const { return ignoreCache; }

    // Random roll
    void randomRoll(uint32_t minRoll = 1, uint32_t maxRoll = 100);

    // Battleground queue slot (aliased from handler_types.hpp)
    using BgQueueSlot = game::BgQueueSlot;

    // Available BG list (aliased from handler_types.hpp)
    using AvailableBgInfo = game::AvailableBgInfo;

    // Battleground
    bool hasPendingBgInvite() const;
    void acceptBattlefield(uint32_t queueSlot = 0xFFFFFFFF);
    void declineBattlefield(uint32_t queueSlot = 0xFFFFFFFF);
    const std::array<BgQueueSlot, 3>& getBgQueues() const;
    const std::vector<AvailableBgInfo>& getAvailableBgs() const;

    // BG scoreboard (aliased from handler_types.hpp)
    using BgPlayerScore = game::BgPlayerScore;
    using ArenaTeamScore = game::ArenaTeamScore;
    using BgScoreboardData = game::BgScoreboardData;
    void requestPvpLog();
    const BgScoreboardData* getBgScoreboard() const;

    // BG flag carrier positions (aliased from handler_types.hpp)
    using BgPlayerPosition = game::BgPlayerPosition;
    const std::vector<BgPlayerPosition>& getBgPlayerPositions() const;

    // Network latency (milliseconds, updated each PONG response)
    uint32_t getLatencyMs() const { return lastLatency; }

    // Logout commands. exitAfterLogout: /quit and /exit leave the game; /logout and
    // /camp drop back to character select.
    void requestLogout(bool exitAfterLogout = false);

    /// Turn the character to face a canonical yaw, and tell the server.
    ///
    /// Setting movementInfo.orientation and sending MSG_MOVE_SET_FACING is not
    /// enough on its own: the frame loop resyncs orientation from the renderer's
    /// character facing every frame and re-sends it once it differs by more than
    /// three degrees, so a facing that exists only in the packet is undone
    /// before anything with a cast time completes. Turning the character makes
    /// the two agree, and keeps them agreeing.
    void faceCanonicalYaw(float canonicalYaw);
    void cancelLogout();

    // Instance difficulty
    void sendSetDifficulty(uint32_t difficulty);
    bool  isLoggingOut() const;
    float getLogoutCountdown() const;

    // Stand state
    void setStandState(uint8_t state);  // 0=stand, 1=sit, 2=sit_chair, 3=sleep, 4=sit_low_chair, 5=sit_medium_chair, 6=sit_high_chair, 7=dead, 8=kneel, 9=submerged
    uint8_t getStandState() const { return standState_; }
    bool isSitting() const { return standState_ >= 1 && standState_ <= 6; }
    bool isDead() const { return standState_ == 7; }
    bool isKneeling() const { return standState_ == 8; }

    // Display toggles
    void toggleHelm();
    void toggleCloak();
    bool isHelmVisible() const { return helmVisible_; }
    bool isCloakVisible() const { return cloakVisible_; }

    // Follow/Assist
    void followTarget();
    void cancelFollow();   // Stop following current target
    void assistTarget();

    // PvP
    void togglePvp();

    // Minimap ping (Ctrl+click on minimap; wowX/wowY in canonical WoW coords)
    void sendMinimapPing(float wowX, float wowY);

    // Guild commands
    void requestGuildInfo();
    void requestGuildRoster();
    void setGuildMotd(const std::string& motd);
    void promoteGuildMember(const std::string& playerName);
    void demoteGuildMember(const std::string& playerName);
    void leaveGuild();
    void inviteToGuild(const std::string& playerName);
    void kickGuildMember(const std::string& playerName);
    void disbandGuild();
    void setGuildLeader(const std::string& name);
    void setGuildPublicNote(const std::string& name, const std::string& note);
    void setGuildOfficerNote(const std::string& name, const std::string& note);
    void acceptGuildInvite();
    void declineGuildInvite();

    // GM Ticket
    void submitGmTicket(const std::string& text);
    void deleteGmTicket();
    void requestGmTicket();          ///< Send CMSG_GMTICKET_GETTICKET to query open ticket

    // GM ticket status accessors
    bool hasActiveGmTicket() const { return gmTicketActive_; }
    const std::string& getGmTicketText() const { return gmTicketText_; }
    bool isGmSupportAvailable() const { return gmSupportAvailable_; }
    float getGmTicketWaitHours() const { return gmTicketWaitHours_; }

    // Battlefield Manager (Wintergrasp)
    bool hasBfMgrInvite()  const { return bfMgrInvitePending_; }
    bool isInBfMgrZone()   const { return bfMgrActive_; }
    uint32_t getBfMgrZoneId() const { return bfMgrZoneId_; }
    void acceptBfMgrInvite();
    void declineBfMgrInvite();

    // WotLK Calendar
    uint32_t getCalendarPendingInvites() const { return calendarPendingInvites_; }
    void requestCalendar(); ///< Send CMSG_CALENDAR_GET_CALENDAR to the server
    void queryGuildInfo(uint32_t guildId);
    void createGuild(const std::string& guildName);
    void addGuildRank(const std::string& rankName);
    void deleteGuildRank();
    void requestPetitionShowlist(uint64_t npcGuid);
    void buyPetition(uint64_t npcGuid, const std::string& guildName);

    // Guild state accessors
    bool isInGuild() const;
    const std::string& getGuildName() const;
    const GuildRosterData& getGuildRoster() const;
    bool hasGuildRoster() const;
    const std::vector<std::string>& getGuildRankNames() const;
    bool hasPendingGuildInvite() const;
    const std::string& getPendingGuildInviterName() const;
    const std::string& getPendingGuildInviteGuildName() const;
    const GuildInfoData& getGuildInfoData() const;
    const GuildQueryResponseData& getGuildQueryData() const;
    bool hasGuildInfoData() const;
    bool hasPetitionShowlist() const;
    void clearPetitionDialog() { showPetitionDialog_ = false; }
    uint32_t getPetitionCost() const;
    uint64_t getPetitionNpcGuid() const;

    // Petition signatures (guild charter signing flow)
    using PetitionSignature = game::PetitionSignature;
    using PetitionInfo = game::PetitionInfo;
    const PetitionInfo& getPetitionInfo() const;
    bool hasPetitionSignaturesUI() const;
    void clearPetitionSignaturesUI() { petitionInfo_.showUI = false; }
    void signPetition(uint64_t petitionGuid);
    void turnInPetition(uint64_t petitionGuid);

    // Guild name lookup for other players' nameplates
    // Returns the guild name for a given guildId, or empty if unknown.
    // Automatically queries the server for unknown guild IDs.
    const std::string& lookupGuildName(uint32_t guildId);
    // Returns the guildId for a player entity (from PLAYER_GUILDID update field).
    uint32_t getEntityGuildId(uint64_t guid) const;

    using ReadyCheckResult = game::ReadyCheckResult;
    void initiateReadyCheck();
    void respondToReadyCheck(bool ready);
    bool hasPendingReadyCheck() const;
    void dismissReadyCheck() { pendingReadyCheck_ = false; }
    const std::string& getReadyCheckInitiator() const;
    const std::vector<ReadyCheckResult>& getReadyCheckResults() const;

    // Duel
    void forfeitDuel();

    // AFK/DND status
    void toggleAfk(const std::string& message = "");
    void toggleDnd(const std::string& message = "");
    bool isAfk() const { return afkStatus_; }
    bool isDnd() const { return dndStatus_; }
    void replyToLastWhisper(const std::string& message);
    std::string getLastWhisperSender() const {
        if (!lastWhisperSender_.empty()) return lastWhisperSender_;
        // Name may not have been cached when whisper arrived — resolve from GUID
        if (lastWhisperSenderGuid_ != 0) {
            const auto& cache = getPlayerNameCache();
            auto it = cache.find(lastWhisperSenderGuid_);
            if (it != cache.end()) return it->second;
        }
        return "";
    }
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
    void reportPlayer(uint64_t targetGuid, const std::string& reason);
    void stopCasting();
    void resetCastState();       // force-clear all cast/craft/queue state without sending packets
    void resetWardenState();     // clear all warden module/crypto state for connect/disconnect
    void clearUnitCaches();      // clear per-unit cast states and aura caches

    void queryPlayerName(uint64_t guid);

    /// Who a piece of mail is from, resolved from its GUID or entry depending on
    /// the mail type. Resolved on demand so a late name query still shows.
    std::string getMailSenderName(const MailMessage& mail) const;
    void queryCreatureInfo(uint32_t entry, uint64_t guid);
    void queryGameObjectInfo(uint32_t entry, uint64_t guid);
    const GameObjectQueryResponseData* getCachedGameObjectInfo(uint32_t entry) const {
        return entityController_->getCachedGameObjectInfo(entry);
    }
    std::string getCachedPlayerName(uint64_t guid) const;
    std::string getCachedCreatureName(uint32_t entry) const;
    // Read-only cache access forwarded from EntityController
    const std::unordered_map<uint64_t, std::string>& getPlayerNameCache() const {
        return entityController_->getPlayerNameCache();
    }
    const std::unordered_map<uint32_t, CreatureQueryResponseData>& getCreatureInfoCache() const {
        return entityController_->getCreatureInfoCache();
    }
    // Returns the creature subname/title (e.g. "<Warchief of the Horde>"), empty if not cached
    std::string getCachedCreatureSubName(uint32_t entry) const {
        return entityController_->getCachedCreatureSubName(entry);
    }
    // Returns the creature rank (0=Normal,1=Elite,2=RareElite,3=Boss,4=Rare)
    // or -1 if not cached yet
    int getCreatureRank(uint32_t entry) const {
        return entityController_->getCreatureRank(entry);
    }
    // Returns creature type (1=Beast,2=Dragonkin,...,7=Humanoid,...) or 0 if not cached
    uint32_t getCreatureType(uint32_t entry) const {
        return entityController_->getCreatureType(entry);
    }
    // Returns creature family (e.g. pet family for beasts) or 0
    uint32_t getCreatureFamily(uint32_t entry) const {
        return entityController_->getCreatureFamily(entry);
    }

    void startAutoAttack(uint64_t targetGuid);
    void stopAutoAttack();
    bool isAutoAttacking() const;
    bool hasAutoAttackIntent() const;
    bool isInCombat() const;
    bool isInCombatWith(uint64_t guid) const;
    uint64_t getAutoAttackTargetGuid() const;
    bool isAggressiveTowardPlayer(uint64_t guid) const;
    // Timestamp (ms since epoch) of the most recent player melee auto-attack.
    // Zero if no swing has occurred this session.
    uint64_t getLastMeleeSwingMs() const;
    const std::vector<CombatTextEntry>& getCombatText() const;
    void clearCombatText();
    void updateCombatText(float deltaTime);
    void clearHostileAttackers();

    // Combat log (persistent rolling history, max MAX_COMBAT_LOG entries)
    const std::deque<CombatLogEntry>& getCombatLog() const;
    void clearCombatLog();

    // Area trigger messages (SMSG_AREA_TRIGGER_MESSAGE) — drained by UI each frame
    bool hasAreaTriggerMsg() const { return !areaTriggerMsgs_.empty(); }
    std::string popAreaTriggerMsg() {
        if (areaTriggerMsgs_.empty()) return {};
        std::string msg = areaTriggerMsgs_.front();
        areaTriggerMsgs_.pop_front();
        return msg;
    }

    // Threat
    using ThreatEntry = CombatHandler::ThreatEntry;
    const std::vector<ThreatEntry>* getThreatList(uint64_t unitGuid) const;
    const std::vector<ThreatEntry>* getTargetThreatList() const;

    void castSpell(uint32_t spellId, uint64_t targetGuid = 0);
    void cancelCast();
    void cancelAura(uint32_t spellId);
    void dismissPet();
    void renamePet(const std::string& newName);
    bool hasPet() const { return petGuid_ != 0; }
    // Returns true once after SMSG_PET_RENAMEABLE; consuming the flag clears it.
    bool consumePetRenameablePending() { bool v = petRenameablePending_; petRenameablePending_ = false; return v; }
    uint64_t getPetGuid() const { return petGuid_; }

    // ---- Pet state (populated by SMSG_PET_SPELLS / SMSG_PET_MODE) ----
    // 10 action bar slots; each entry is a packed uint32:
    //   bits 0-23  = spell ID (or 0 for empty)
    //   bits 24-31 = action type (0x00=cast, 0xC0=autocast on, 0x40=autocast off)
    static constexpr int PET_ACTION_BAR_SLOTS = 10;
    uint32_t getPetActionSlot(int idx) const {
        if (idx < 0 || idx >= PET_ACTION_BAR_SLOTS) return 0;
        return petActionSlots_[idx];
    }
    // Pet command/react state from SMSG_PET_MODE or SMSG_PET_SPELLS
    uint8_t getPetCommand() const { return petCommand_; }   // 0=stay,1=follow,2=attack,3=dismiss
    uint8_t getPetReact()   const { return petReact_; }     // 0=passive,1=defensive,2=aggressive
    // Spells the pet knows (from SMSG_PET_SPELLS spell list)
    const std::vector<uint32_t>& getPetSpells() const { return petSpellList_; }
    // Pet autocast set (spellIds that have autocast enabled)
    bool isPetSpellAutocast(uint32_t spellId) const {
        return petAutocastSpells_.count(spellId) != 0;
    }
    // Send CMSG_PET_ACTION to issue a pet command
    void sendPetAction(uint32_t action, uint64_t targetGuid = 0);
    // Toggle autocast for a pet spell via CMSG_PET_SPELL_AUTOCAST
    void togglePetSpellAutocast(uint32_t spellId);
    const std::unordered_set<uint32_t>& getKnownSpells() const {
        static const std::unordered_set<uint32_t> empty;
        return spellHandler_ ? spellHandler_->getKnownSpells() : empty;
    }

    // Spell book tabs — groups known spells by class skill line for Lua API
    using SpellBookTab = SpellHandler::SpellBookTab;
    const std::vector<SpellBookTab>& getSpellBookTabs();

    // ---- Pet Stable ----
    struct StabledPet {
        uint32_t petNumber  = 0;   // server-side pet number (used for unstable/swap)
        uint32_t entry      = 0;   // creature entry ID
        uint32_t level      = 0;
        std::string name;
        uint32_t displayId  = 0;
        bool     isActive   = false;  // true = currently summoned/active slot
    };
    bool isStableWindowOpen() const { return stableWindowOpen_; }
    void closeStableWindow() { stableWindowOpen_ = false; }
    uint64_t getStableMasterGuid() const { return stableMasterGuid_; }
    uint8_t  getStableSlots() const { return stableNumSlots_; }
    const std::vector<StabledPet>& getStabledPets() const { return stabledPets_; }
    void requestStabledPetList();          // CMSG MSG_LIST_STABLED_PETS
    void stablePet(uint8_t slot);          // CMSG_STABLE_PET (store active pet in slot)
    void unstablePet(uint32_t petNumber);  // CMSG_UNSTABLE_PET (retrieve to active)

    // Player proficiency bitmasks (from SMSG_SET_PROFICIENCY)
    // itemClass 2 = Weapon (subClassMask bits: 0=Axe1H,1=Axe2H,2=Bow,3=Gun,4=Mace1H,5=Mace2H,6=Polearm,7=Sword1H,8=Sword2H,10=Staff,13=Fist,14=Misc,15=Dagger,16=Thrown,17=Crossbow,18=Wand,19=Fishing)
    // itemClass 4 = Armor (subClassMask bits: 1=Cloth,2=Leather,3=Mail,4=Plate,6=Shield)
    uint32_t getWeaponProficiency() const { return weaponProficiency_; }
    uint32_t getArmorProficiency()  const { return armorProficiency_; }
    bool canUseWeaponSubclass(uint32_t subClass) const { return (weaponProficiency_ >> subClass) & 1u; }
    bool canUseArmorSubclass(uint32_t subClass)  const { return (armorProficiency_  >> subClass) & 1u; }

    // Minimap pings from party members
    struct MinimapPing {
        uint64_t senderGuid = 0;
        float    wowX       = 0.0f;  // canonical WoW X (north)
        float    wowY       = 0.0f;  // canonical WoW Y (west)
        float    age        = 0.0f;  // seconds since received
        static constexpr float LIFETIME = 5.0f;
        bool isExpired() const { return age >= LIFETIME; }
    };
    const std::vector<MinimapPing>& getMinimapPings() const { return minimapPings_; }
    void tickMinimapPings(float dt) {
        for (auto& p : minimapPings_) p.age += dt;
        minimapPings_.erase(
            std::remove_if(minimapPings_.begin(), minimapPings_.end(),
                           [](const MinimapPing& p){ return p.isExpired(); }),
            minimapPings_.end());
    }

    bool isCasting() const { return spellHandler_ ? spellHandler_->isCasting() : false; }
    bool isChanneling() const { return spellHandler_ ? spellHandler_->isChanneling() : false; }
    bool isRestoring() const { return spellHandler_ ? spellHandler_->isRestoring() : false; }
    bool isGameObjectInteractionCasting() const {
        return spellHandler_ ? spellHandler_->isGameObjectInteractionCasting() : false;
    }
    uint32_t getCurrentCastSpellId() const { return spellHandler_ ? spellHandler_->getCurrentCastSpellId() : 0; }
    float getCastProgress() const { return spellHandler_ ? spellHandler_->getCastProgress() : 0.0f; }
    float getCastTimeRemaining() const { return spellHandler_ ? spellHandler_->getCastTimeRemaining() : 0.0f; }
    float getCastTimeTotal() const { return spellHandler_ ? spellHandler_->getCastTimeTotal() : 0.0f; }

    // Repeat-craft queue
    void startCraftQueue(uint32_t spellId, int count);
    void cancelCraftQueue();
    int getCraftQueueRemaining() const;
    uint32_t getCraftQueueSpellId() const;

    // Crafting window (opened client-side by casting a profession spell)
    bool isCraftingWindowOpen() const { return spellHandler_ ? spellHandler_->isCraftingWindowOpen() : false; }
    uint32_t getCraftingSkillLine() const { return spellHandler_ ? spellHandler_->getCraftingSkillLine() : 0; }
    void closeCraftingWindow() { if (spellHandler_) spellHandler_->closeCraftingWindow(); }

    // 400ms spell-queue window: next spell to cast when current finishes
    uint32_t getQueuedSpellId() const;
    void cancelQueuedSpell() { if (spellHandler_) spellHandler_->cancelQueuedSpell(); }

    // Unit cast state (aliased from handler_types.hpp)
    using UnitCastState = game::UnitCastState;
    // Returns cast state for any unit by GUID (delegates to SpellHandler)
    const UnitCastState* getUnitCastState(uint64_t guid) const {
        if (spellHandler_) return spellHandler_->getUnitCastState(guid);
        return nullptr;
    }
    // Convenience helpers for the current target
    bool isTargetCasting() const { return spellHandler_ ? spellHandler_->isTargetCasting() : false; }
    uint32_t getTargetCastSpellId() const { return spellHandler_ ? spellHandler_->getTargetCastSpellId() : 0; }
    float getTargetCastProgress() const { return spellHandler_ ? spellHandler_->getTargetCastProgress() : 0.0f; }
    float getTargetCastTimeRemaining() const { return spellHandler_ ? spellHandler_->getTargetCastTimeRemaining() : 0.0f; }
    bool isTargetCastInterruptible() const { return spellHandler_ ? spellHandler_->isTargetCastInterruptible() : true; }

    // Talents — delegate to SpellHandler as canonical authority
    uint8_t getActiveTalentSpec() const { return spellHandler_ ? spellHandler_->getActiveTalentSpec() : 0; }
    uint8_t getUnspentTalentPoints() const { return spellHandler_ ? spellHandler_->getUnspentTalentPoints() : 0; }
    uint8_t getUnspentTalentPoints(uint8_t spec) const { return spellHandler_ ? spellHandler_->getUnspentTalentPoints(spec) : 0; }
    const std::unordered_map<uint32_t, uint8_t>& getLearnedTalents() const {
        if (spellHandler_) return spellHandler_->getLearnedTalents();
        static const std::unordered_map<uint32_t, uint8_t> empty;
        return empty;
    }
    const std::unordered_map<uint32_t, uint8_t>& getLearnedTalents(uint8_t spec) const {
        if (spellHandler_) return spellHandler_->getLearnedTalents(spec);
        static const std::unordered_map<uint32_t, uint8_t> empty;
        return empty;
    }

    // Glyphs (WotLK): up to 6 glyph slots per spec (3 major + 3 minor)
    static constexpr uint8_t MAX_GLYPH_SLOTS = 6;
    const std::array<uint16_t, MAX_GLYPH_SLOTS>& getGlyphs() const {
        if (spellHandler_) return spellHandler_->getGlyphs();
        static const std::array<uint16_t, MAX_GLYPH_SLOTS> empty{};
        return empty;
    }
    const std::array<uint16_t, MAX_GLYPH_SLOTS>& getGlyphs(uint8_t spec) const {
        if (spellHandler_) return spellHandler_->getGlyphs(spec);
        static const std::array<uint16_t, MAX_GLYPH_SLOTS> empty{};
        return empty;
    }
    uint8_t getTalentRank(uint32_t talentId) const {
        return spellHandler_ ? spellHandler_->getTalentRank(talentId) : 0;
    }
    void learnTalent(uint32_t talentId, uint32_t requestedRank);
    void switchTalentSpec(uint8_t newSpec);

    // Talent DBC access
    const TalentEntry* getTalentEntry(uint32_t talentId) const {
        if (spellHandler_) return spellHandler_->getTalentEntry(talentId);
        auto it = talentCache_.find(talentId);
        return (it != talentCache_.end()) ? &it->second : nullptr;
    }
    const TalentTabEntry* getTalentTabEntry(uint32_t tabId) const {
        if (spellHandler_) return spellHandler_->getTalentTabEntry(tabId);
        auto it = talentTabCache_.find(tabId);
        return (it != talentTabCache_.end()) ? &it->second : nullptr;
    }
    const std::unordered_map<uint32_t, TalentEntry>& getAllTalents() const;
    const std::unordered_map<uint32_t, TalentTabEntry>& getAllTalentTabs() const;
    void loadTalentDbc();

    // Action bar — 12 pages × 12 slots = 144 total.
    // The first 6 pages match FrameXML action pages:
    // Page 1: main bar, pages 2-6: scrollable main pages / fixed multi-bars.
    // TBC sends 132 slots; WotLK sends 144.  Keep the full WotLK-sized array so
    // later pages are not discarded when loading server action buttons.
    static constexpr int SLOTS_PER_BAR    = 12;
    static constexpr int ACTION_BARS      = 12;
    static constexpr int ACTION_BAR_SLOTS = SLOTS_PER_BAR * ACTION_BARS;   // 144
    std::array<ActionBarSlot, ACTION_BAR_SLOTS>& getActionBar() { return actionBar; }
    const std::array<ActionBarSlot, ACTION_BAR_SLOTS>& getActionBar() const { return actionBar; }
    void setActionBarSlot(int slot, ActionBarSlot::Type type, uint32_t id);

    // Client-side macro text storage (server sends only macro index; text is stored locally)
    const std::string& getMacroText(uint32_t macroId) const;
    void setMacroText(uint32_t macroId, const std::string& text);

    void saveCharacterConfig();
    void loadCharacterConfig();
    static std::string getCharacterConfigDir();

    // Auras — delegate to SpellHandler as canonical authority
    const std::vector<AuraSlot>& getPlayerAuras() const {
        if (spellHandler_) return spellHandler_->getPlayerAuras();
        static const std::vector<AuraSlot> empty;
        return empty;
    }
    const std::vector<AuraSlot>& getTargetAuras() const {
        if (spellHandler_) return spellHandler_->getTargetAuras();
        static const std::vector<AuraSlot> empty;
        return empty;
    }
    // Per-unit aura cache (populated for party members and any unit we receive updates for)
    const std::vector<AuraSlot>* getUnitAuras(uint64_t guid) const {
        if (spellHandler_) return spellHandler_->getUnitAuras(guid);
        return nullptr;
    }

    // Completed quests (populated from SMSG_QUERY_QUESTS_COMPLETED_RESPONSE)
    bool isQuestCompleted(uint32_t questId) const { return completedQuests_.count(questId) > 0; }
    const std::unordered_set<uint32_t>& getCompletedQuests() const { return completedQuests_; }

    // NPC death callback (for animations)
    using NpcDeathCallback = std::function<void(uint64_t guid)>;
    void setNpcDeathCallback(NpcDeathCallback cb) { npcDeathCallback_ = std::move(cb); }

    // Resolves a unit GUID to its CharacterRenderer instance id (0 = not
    // rendered). Wired by AnimationCallbackHandler; used to bone-attach spell
    // visuals to the actual caster instead of the local player.
    using UnitRenderInstanceResolver = std::function<uint32_t(uint64_t guid)>;
    void setUnitRenderInstanceResolver(UnitRenderInstanceResolver cb) { unitRenderInstanceResolver_ = std::move(cb); }
    uint32_t resolveUnitRenderInstance(uint64_t guid) const {
        return unitRenderInstanceResolver_ ? unitRenderInstanceResolver_(guid) : 0;
    }

    using NpcAggroCallback = std::function<void(uint64_t guid, const glm::vec3& position)>;
    void setNpcAggroCallback(NpcAggroCallback cb) { npcAggroCallback_ = std::move(cb); }

    // NPC respawn callback (health 0 → >0, resets animation to idle)
    using NpcRespawnCallback = std::function<void(uint64_t guid)>;
    void setNpcRespawnCallback(NpcRespawnCallback cb) { npcRespawnCallback_ = std::move(cb); }

    // Stand state animation callback — fired when SMSG_STANDSTATE_UPDATE confirms a new state
    // standState: 0=stand, 1-6=sit variants, 7=dead, 8=kneel
    using StandStateCallback = std::function<void(uint8_t standState)>;
    void setStandStateCallback(StandStateCallback cb) { standStateCallback_ = std::move(cb); }

    // Logout complete callback — fired when SMSG_LOGOUT_COMPLETE says the character
    // is out of the world. exiting is true for /quit and /exit (leave the game),
    // false for /logout and /camp (back to character select).
    using LogoutCompleteCallback = std::function<void(bool exiting)>;
    void setLogoutCompleteCallback(LogoutCompleteCallback cb) { logoutCompleteCallback_ = std::move(cb); }
    auto& logoutCompleteCallbackRef() { return logoutCompleteCallback_; }

    // Appearance changed callback — fired when PLAYER_BYTES or facial features update (barber shop, etc.)
    using AppearanceChangedCallback = std::function<void()>;
    void setAppearanceChangedCallback(AppearanceChangedCallback cb) { appearanceChangedCallback_ = std::move(cb); }

    // Player model rebuild callback — fired alongside the appearance change so the
    // in-world 3D character can be respawned with the new hair/facial hair without
    // requiring a game restart.
    using PlayerModelRebuildCallback = std::function<void()>;
    void setPlayerModelRebuildCallback(PlayerModelRebuildCallback cb) { playerModelRebuildCallback_ = std::move(cb); }

    // Ghost state callback — fired when player enters or leaves ghost (spirit) form
    using GhostStateCallback = std::function<void(bool isGhost)>;
    void setGhostStateCallback(GhostStateCallback cb) { ghostStateCallback_ = std::move(cb); }

    // Melee swing callback (for driving animation/SFX)
    // spellId: 0 = regular auto-attack swing, non-zero = melee ability (special attack)
    using MeleeSwingCallback = std::function<void(uint32_t spellId)>;
    void setMeleeSwingCallback(MeleeSwingCallback cb) { meleeSwingCallback_ = std::move(cb); }

    // Snap the character to face the camera's current look direction and return the
    // resulting canonical orientation. Used by fishing so the bobber lands in front of
    // where the player is looking: while standing still the character yaw does not track
    // the free-look camera, so without this the bobber would drop toward a stale heading.
    using FaceCameraProvider = std::function<float()>;
    void setFaceCameraProvider(FaceCameraProvider cb) { faceCameraProvider_ = std::move(cb); }
    float faceCameraDirection() {
        return faceCameraProvider_ ? faceCameraProvider_() : getMovementInfo().orientation;
    }

    // Ranged weapon swap callback — show=true: swap to ranged weapon, false: back to melee
    using RangedWeaponSwapCallback = std::function<void(bool show)>;
    void setRangedWeaponSwapCallback(RangedWeaponSwapCallback cb) { rangedWeaponSwapCallback_ = std::move(cb); }

    // Spell cast animation callbacks — true=start cast/channel, false=finish/cancel
    // guid: caster (may be player or another unit), isChannel: channel vs regular cast
    // castType: DIRECTED (unit target), OMNI (self/no target), AREA (ground AoE)
    using SpellCastAnimCallback = std::function<void(uint64_t guid, bool start, bool isChannel,
                                                      SpellCastType castType)>;
    void setSpellCastAnimCallback(SpellCastAnimCallback cb) { spellCastAnimCallback_ = std::move(cb); }

    // Fired when the player's own spell cast fails (spellId of the failed spell).
    using SpellCastFailedCallback = std::function<void(uint32_t spellId)>;
    void setSpellCastFailedCallback(SpellCastFailedCallback cb) { spellCastFailedCallback_ = std::move(cb); }

    // Unit animation hint: signal jump (animId=38) for other players/NPCs
    using UnitAnimHintCallback = std::function<void(uint64_t guid, uint32_t animId)>;
    void setUnitAnimHintCallback(UnitAnimHintCallback cb) { unitAnimHintCallback_ = std::move(cb); }

    // Unit move-flags callback: fired on every MSG_MOVE_* for other players with the raw flags field.
    // Drives Walk(4) vs Run(5) selection and swim state initialization from heartbeat packets.
    using UnitMoveFlagsCallback = std::function<void(uint64_t guid, uint32_t moveFlags)>;
    void setUnitMoveFlagsCallback(UnitMoveFlagsCallback cb) { unitMoveFlagsCallback_ = std::move(cb); }

    // NPC swing callback (plays attack animation on NPC)
    using NpcSwingCallback = std::function<void(uint64_t guid)>;
    void setNpcSwingCallback(NpcSwingCallback cb) { npcSwingCallback_ = std::move(cb); }

    // Hit reaction callback — triggers victim animation (dodge, block, wound, crit wound)
    enum class HitReaction : uint8_t { WOUND, CRIT_WOUND, DODGE, PARRY, BLOCK, SHIELD_BLOCK };
    using HitReactionCallback = std::function<void(uint64_t victimGuid, HitReaction reaction)>;
    void setHitReactionCallback(HitReactionCallback cb) { hitReactionCallback_ = std::move(cb); }

    // Stun state callback — fires when UNIT_FLAG_STUNNED changes on the local player
    using StunStateCallback = std::function<void(bool stunned)>;
    void setStunStateCallback(StunStateCallback cb) { stunStateCallback_ = std::move(cb); }

    // Stealth state callback — fires when UNIT_FLAG_SNEAKING changes on the local player
    using StealthStateCallback = std::function<void(bool stealthed)>;
    void setStealthStateCallback(StealthStateCallback cb) { stealthStateCallback_ = std::move(cb); }

    // Player health changed callback — fires when local player HP changes
    using PlayerHealthCallback = std::function<void(uint32_t health, uint32_t maxHealth)>;
    void setPlayerHealthCallback(PlayerHealthCallback cb) { playerHealthCallback_ = std::move(cb); }

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
    uint32_t getPlayerRestedXp() const { return playerRestedXp_; }
    bool isPlayerResting() const { return isResting_; }
    uint32_t getPlayerLevel() const { return serverPlayerLevel_; }
    const std::vector<uint32_t>& getPlayerExploredZoneMasks() const { return playerExploredZones_; }
    bool hasPlayerExploredZoneMasks() const { return hasPlayerExploredZones_; }
    static uint32_t killXp(uint32_t playerLevel, uint32_t victimLevel);

    // Server game time, in HOURS since midnight (0.0-24.0).
    //
    // The unit is stated because four things disagreed about it: the wire
    // format is a packed bitfield, the sky divided by 86400 as though it were
    // seconds, GetGameTime split it as hours, and SMSG_SERVERTIME wrote a unix
    // timestamp into the same field.
    float getGameTime() const { return gameTime_; }
    float getTimeSpeed() const { return timeSpeed_; }

    // Global Cooldown (GCD) — set when the server sends a spellId=0 cooldown entry
    float getGCDRemaining() const {
        if (gcdTotal_ <= 0.0f) return 0.0f;
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - gcdStartedAt_).count() / 1000.0f;
        float rem = gcdTotal_ - elapsed;
        return rem > 0.0f ? rem : 0.0f;
    }
    float getGCDTotal() const;
    bool isGCDActive() const { return getGCDRemaining() > 0.0f; }

    // Weather state (updated by SMSG_WEATHER)
    // weatherType: 0=clear, 1=rain, 2=snow, 3=storm/fog
    uint32_t getWeatherType() const { return weatherType_; }
    float getWeatherIntensity() const { return weatherIntensity_; }
    bool isRaining() const { return weatherType_ == 1 && weatherIntensity_ > 0.05f; }
    bool isSnowing() const { return weatherType_ == 2 && weatherIntensity_ > 0.05f; }
    uint32_t getOverrideLightId() const { return overrideLightId_; }
    uint32_t getOverrideLightTransMs() const { return overrideLightTransMs_; }

    // Player skills
    const std::unordered_map<uint32_t, PlayerSkill>& getPlayerSkills() const { return playerSkills_; }
    const std::string& getSkillName(uint32_t skillId) const;
    uint32_t getSkillCategory(uint32_t skillId) const;
    bool isProfessionSpell(uint32_t spellId) const;

    // World entry callback (online mode - triggered when entering world)
    // Parameters: mapId, x, y, z (canonical WoW coords), isInitialEntry=true on first login or reconnect
    using WorldEntryCallback = std::function<void(uint32_t mapId, float x, float y, float z, bool isInitialEntry)>;
    void setWorldEntryCallback(WorldEntryCallback cb) { worldEntryCallback_ = std::move(cb); }

    // Knockback callback: called when server sends SMSG_MOVE_KNOCK_BACK for the player.
    // Parameters: vcos, vsin (2D direction vector in server/wire coord space — the
    //   server→canonical→render swaps cancel, so the consumer can use them directly
    //   in render space, see CameraController::applyKnockBack),
    //   hspeed, vspeed (raw from packet; vspeed is negative when the server intends
    //   an upward launch — negate before applying as initial Y velocity).
    using KnockBackCallback = std::function<void(float vcos, float vsin, float hspeed, float vspeed)>;
    void setKnockBackCallback(KnockBackCallback cb) { knockBackCallback_ = std::move(cb); }

    // Camera shake callback: called when server sends SMSG_CAMERA_SHAKE.
    // Parameters: magnitude (world units), frequency (Hz), duration (seconds).
    using CameraShakeCallback = std::function<void(float magnitude, float frequency, float duration)>;
    void setCameraShakeCallback(CameraShakeCallback cb) { cameraShakeCallback_ = std::move(cb); }

    // Auto-follow callback: pass render-space position pointer to start, nullptr to cancel.
    using AutoFollowCallback = std::function<void(const glm::vec3* renderPos)>;
    void setAutoFollowCallback(AutoFollowCallback cb) { autoFollowCallback_ = std::move(cb); }

    // Unstuck callback (resets player Z to floor height)
    using UnstuckCallback = std::function<void()>;
    void setUnstuckCallback(UnstuckCallback cb) { unstuckCallback_ = std::move(cb); }
    void unstuck();
    void setUnstuckGyCallback(UnstuckCallback cb) { unstuckGyCallback_ = std::move(cb); }
    void unstuckGy();
    void setUnstuckHearthCallback(UnstuckCallback cb) { unstuckHearthCallback_ = std::move(cb); }
    void unstuckHearth();
    using BindPointCallback = std::function<void(uint32_t mapId, float x, float y, float z)>;
    void setBindPointCallback(BindPointCallback cb) { bindPointCallback_ = std::move(cb); }

    // Called when the player starts casting Hearthstone so terrain at the bind
    // point can be pre-loaded during the cast time.
    // Parameters: mapId and canonical (x, y, z) of the bind location.
    using HearthstonePreloadCallback = std::function<void(uint32_t mapId, float x, float y, float z)>;
    void setHearthstonePreloadCallback(HearthstonePreloadCallback cb) { hearthstonePreloadCallback_ = std::move(cb); }

    // Creature spawn callback (online mode - triggered when creature enters view)
    // Parameters: guid, displayId, x, y, z (canonical), orientation, scale (OBJECT_FIELD_SCALE_X)
    using CreatureSpawnCallback = std::function<void(uint64_t guid, uint32_t displayId, float x, float y, float z, float orientation, float scale)>;
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
    // Parameters: guid, entry, displayId, x, y, z (canonical), orientation, scale (OBJECT_FIELD_SCALE_X)
    using GameObjectSpawnCallback = std::function<void(uint64_t guid, uint32_t entry, uint32_t displayId, float x, float y, float z, float orientation, float scale)>;
    void setGameObjectSpawnCallback(GameObjectSpawnCallback cb) { gameObjectSpawnCallback_ = std::move(cb); }

    // GameObject move callback (online mode - triggered when gameobject position updates)
    // Parameters: guid, x, y, z (canonical), orientation
    using GameObjectMoveCallback = std::function<void(uint64_t guid, float x, float y, float z, float orientation)>;
    void setGameObjectMoveCallback(GameObjectMoveCallback cb) { gameObjectMoveCallback_ = std::move(cb); }

    // GameObject metadata callback (triggered when a GAMEOBJECT_QUERY_RESPONSE is
    // cached). A game object's model is spawned from its display id, which arrives
    // before its type does, so anything that depends on the type — such as whether
    // the model should animate — has to be revisited when this fires.
    using GameObjectInfoCallback = std::function<void(uint32_t entry)>;
    void setGameObjectInfoCallback(GameObjectInfoCallback cb) { gameObjectInfoCallback_ = std::move(cb); }

    // GameObject despawn callback (online mode - triggered when gameobject leaves view)
    using GameObjectDespawnCallback = std::function<void(uint64_t guid)>;
    void setGameObjectDespawnCallback(GameObjectDespawnCallback cb) { gameObjectDespawnCallback_ = std::move(cb); }

    using GameObjectCustomAnimCallback = std::function<void(uint64_t guid, uint32_t animId)>;
    void setGameObjectCustomAnimCallback(GameObjectCustomAnimCallback cb) { gameObjectCustomAnimCallback_ = std::move(cb); }

    // GameObject state change callback (triggered when GAMEOBJECT_BYTES_1 updates — state byte changes)
    // goState: 0=READY(closed), 1=OPEN, 2=DESTROYED
    using GameObjectStateCallback = std::function<void(uint64_t guid, uint8_t goState)>;
    void setGameObjectStateCallback(GameObjectStateCallback cb) { gameObjectStateCallback_ = std::move(cb); }

    // Sprint aura callback — fired when sprint-type aura active state changes on player
    using SprintAuraCallback = std::function<void(bool active)>;
    void setSprintAuraCallback(SprintAuraCallback cb) { sprintAuraCallback_ = std::move(cb); }

    // Vehicle state callback — fired when player enters/exits a vehicle
    using VehicleStateCallback = std::function<void(bool entered, uint32_t vehicleId)>;
    void setVehicleStateCallback(VehicleStateCallback cb) { vehicleStateCallback_ = std::move(cb); }

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
    bool isTransportGuid(uint64_t guid) const { return entityController_->isTransportGuid(guid); }
    bool hasServerTransportUpdate(uint64_t guid) const { return entityController_->hasServerTransportUpdate(guid); }
    glm::vec3 getComposedWorldPosition();  // Compose transport transform * local offset
    TransportManager* getTransportManager() { return transportManager_.get(); }
    // Client-side transport board/disembark check by proximity to the live deck.
    // Covers M2 trams/lifts and client-animated TaxiPathNode WMO ships.
    void updateM2TransportBoarding(const glm::vec3& playerCanonical);
    void setPlayerOnTransport(uint64_t transportGuid, const glm::vec3& localOffset) {
        // Validate transport is registered before attaching player
        // (defer if transport not yet registered to prevent desyncs)
        if (transportGuid != 0 && !isTransportGuid(transportGuid)) {
            return;  // Transport not yet registered; skip attachment
        }
        playerTransportGuid_ = transportGuid;
        playerTransportOffset_ = localOffset;
        playerTransportStickyGuid_ = transportGuid;
        playerTransportStickyTimer_ = 8.0f;
        movementInfo.transportGuid = transportGuid;
    }
    void setPlayerTransportOffset(const glm::vec3& offset) {
        playerTransportOffset_ = offset;
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
    // Preserve an authoritative on-deck offset while a continent transfer tears
    // down the origin map's transport and constructs its destination instance.
    void beginPlayerTransportWorldTransfer(uint32_t destinationMapId,
                                           const glm::vec3& localOffset);
    bool hasPendingPlayerTransportWorldTransfer() const {
        return pendingPlayerTransportTransfer_;
    }
    bool completePlayerTransportWorldTransfer(uint64_t transportGuid,
                                              glm::vec3& worldPosition);

    // Cooldowns
    float getSpellCooldown(uint32_t spellId) const;
    const std::unordered_map<uint32_t, float>& getSpellCooldowns() const {
        static const std::unordered_map<uint32_t, float> empty;
        return spellHandler_ ? spellHandler_->getSpellCooldowns() : empty;
    }

    // Player GUID
    uint64_t getPlayerGuid() const { return playerGuid; }

    // Look up class/race for a player GUID from name query cache. Returns 0 if unknown.
    uint8_t lookupPlayerClass(uint64_t guid) const {
        return entityController_->lookupPlayerClass(guid);
    }
    uint8_t lookupPlayerRace(uint64_t guid) const {
        return entityController_->lookupPlayerRace(guid);
    }

    // Look up a display name for any guid: checks playerNameCache then entity manager.
    // Returns empty string if unknown. Used by chat display to resolve names at render time.
    const std::string& lookupName(uint64_t guid) const {
        return entityController_->lookupName(guid);
    }

    uint8_t getPlayerClass() const {
        const Character* ch = getActiveCharacter();
        return ch ? static_cast<uint8_t>(ch->characterClass) : 0;
    }
    uint8_t getPlayerRace() const {
        const Character* ch = getActiveCharacter();
        return ch ? static_cast<uint8_t>(ch->race) : 0;
    }
    // Horde races: Orc, Undead, Tauren, Troll, Goblin, Blood Elf. Everything
    // else (including an unknown/zero race) defaults to Alliance.
    bool isPlayerAlliance() const {
        switch (static_cast<Race>(getPlayerRace())) {
            case Race::ORC: case Race::UNDEAD: case Race::TAUREN:
            case Race::TROLL: case Race::GOBLIN: case Race::BLOOD_ELF:
                return false;
            default:
                return true;
        }
    }
    void setPlayerGuid(uint64_t guid) { playerGuid = guid; }

    // Player death state
    bool isPlayerDead() const { return playerDead_; }
    bool isPlayerGhost() const { return releasedSpirit_; }
    bool showDeathDialog() const { return playerDead_ && !releasedSpirit_; }
    bool showResurrectDialog() const { return resurrectRequestPending_; }
    /** True when SMSG_PRE_RESURRECT arrived — Reincarnation/Twisting Nether available. */
    bool canSelfRes() const { return selfResAvailable_; }
    /** Send CMSG_SELF_RES to use Reincarnation / Twisting Nether. */
    void useSelfRes();
    const std::string& getResurrectCasterName() const { return resurrectCasterName_; }
    bool showTalentWipeConfirmDialog() const;
    uint32_t getTalentWipeCost() const;
    void confirmTalentWipe();
    void cancelTalentWipe();
    // Pet talent respec confirm
    bool showPetUnlearnDialog() const;
    uint32_t getPetUnlearnCost() const;
    void confirmPetUnlearn();
    void cancelPetUnlearn();

    // Barber shop
    bool isBarberShopOpen() const { return barberShopOpen_; }
    void closeBarberShop() { barberShopOpen_ = false; fireAddonEvent("BARBER_SHOP_CLOSE", {}); }
    void sendAlterAppearance(uint32_t hairStyleEntry, uint32_t hairColor,
                             uint32_t facialHairEntry, uint32_t skinColorEntry);

    // Instance difficulty (0=5N, 1=5H, 2=25N, 3=25H for WotLK)
    uint32_t getInstanceDifficulty() const;
    bool isInstanceHeroic() const;
    bool isInInstance() const;

    /** True when ghost is within 40 yards of corpse position (same map). */
    bool canReclaimCorpse() const;
    /** Seconds remaining on the PvP corpse-reclaim delay, or 0 if the reclaim is available now. */
    float getCorpseReclaimDelaySec() const;
    /** Distance (yards) from ghost to corpse, or -1 if no corpse data. */
    float getCorpseDistance() const {
        if (!corpsePositionValid_ || currentMapId_ != corpseMapId_) return -1.0f;
        // movementInfo is canonical (x=north=server_y, y=west=server_x);
        // corpse coords are raw server (x=west, y=north) — swap to compare.
        float dx = movementInfo.x - corpseY_;
        float dy = movementInfo.y - corpseX_;
        float dz = movementInfo.z - corpseZ_;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }
    /** Corpse position in canonical WoW coords (X=north, Y=west).
     *  Returns false if no corpse data or on a different map. */
    bool getCorpseCanonicalPos(float& outX, float& outY) const {
        if (!corpsePositionValid_ || currentMapId_ != corpseMapId_) return false;
        outX = corpseY_;  // server Y = canonical X (north)
        outY = corpseX_;  // server X = canonical Y (west)
        return true;
    }
    /** Send CMSG_RECLAIM_CORPSE; noop if not a ghost or not near corpse. */
    void reclaimCorpse();
    void releaseSpirit();
    void acceptResurrect();
    void declineResurrect();

    // ---- Group ----
    void inviteToGroup(const std::string& playerName);
    void acceptGroupInvite();
    void declineGroupInvite();
    void leaveGroup();
    void convertToRaid();
    void sendSetLootMethod(uint32_t method, uint32_t threshold, uint64_t masterLooterGuid);
    bool isInGroup() const;
    const GroupListData& getPartyData() const;
    const std::vector<ContactEntry>& getContacts() const { return contacts_; }
    bool hasPendingGroupInvite() const;
    const std::string& getPendingInviterName() const;

    // ---- Item text (books / readable items) ----
    bool isItemTextOpen() const;
    const std::string& getItemText() const;
    void closeItemText();
    void queryItemText(uint64_t itemGuid);

    // ---- Shared Quest ----
    bool hasPendingSharedQuest() const;
    uint32_t getSharedQuestId() const;
    const std::string& getSharedQuestTitle() const;
    const std::string& getSharedQuestSharerName() const;
    void acceptSharedQuest();
    void declineSharedQuest();

    // ---- Summon ----
    bool hasPendingSummonRequest() const { return pendingSummonRequest_; }
    const std::string& getSummonerName() const { return summonerName_; }
    float getSummonTimeoutSec() const { return summonTimeoutSec_; }
    void acceptSummon();
    void declineSummon();
    void tickSummonTimeout(float dt) {
        if (!pendingSummonRequest_) return;
        summonTimeoutSec_ -= dt;
        if (summonTimeoutSec_ <= 0.0f) {
            pendingSummonRequest_ = false;
            summonTimeoutSec_ = 0.0f;
        }
    }

    // ---- Trade ----
    enum class TradeStatus : uint8_t {
        None = 0, PendingIncoming, Open, Accepted, Complete
    };

    // 7 slots total: 0-5 are transferred, slot 6 (TRADE_SLOT_NONTRADED) is the
    // "will not be traded" slot used for enchanting/crafting on the partner's item.
    static constexpr int TRADE_SLOT_COUNT        = 7;
    static constexpr int TRADE_SLOT_TRADED_COUNT = 6;
    static constexpr int TRADE_SLOT_NONTRADED    = 6;

    struct TradeSlot {
        uint32_t itemId      = 0;
        uint32_t displayId   = 0;
        uint32_t stackCount  = 0;
        uint64_t itemGuid    = 0;
        uint8_t  bag         = 0xFF;   // 0xFF = not set
        uint8_t  bagSlot     = 0xFF;
        bool     occupied    = false;
    };

    TradeStatus getTradeStatus() const;
    bool hasPendingTradeRequest() const;
    bool isTradeOpen() const;
    const std::string& getTradePeerName() const;

    // My trade slots (what I'm offering)
    const std::array<TradeSlot, TRADE_SLOT_COUNT>& getMyTradeSlots() const;
    const std::array<TradeSlot, TRADE_SLOT_COUNT>& getPeerTradeSlots() const;
    uint64_t getMyTradeGold() const;
    uint64_t getPeerTradeGold() const;

    void acceptTradeRequest();   // respond to incoming SMSG_TRADE_STATUS(1) with CMSG_BEGIN_TRADE
    void declineTradeRequest();  // respond with CMSG_CANCEL_TRADE
    void acceptTrade();          // lock in offer: CMSG_ACCEPT_TRADE
    void cancelTrade();          // CMSG_CANCEL_TRADE
    void setTradeItem(uint8_t tradeSlot, uint8_t bag, uint8_t bagSlot);
    void clearTradeItem(uint8_t tradeSlot);
    void setTradeGold(uint64_t copper);

    // ---- Duel ----
    bool hasPendingDuelRequest() const;
    const std::string& getDuelChallengerName() const;
    void acceptDuel();
    // forfeitDuel() already declared at line ~399
    // Returns remaining duel countdown seconds, or 0 if no active countdown
    float getDuelCountdownRemaining() const;

    // Instance lockouts (aliased from handler_types.hpp)
    using InstanceLockout = game::InstanceLockout;
    const std::vector<InstanceLockout>& getInstanceLockouts() const;

    // Boss encounter unit tracking (SMSG_UPDATE_INSTANCE_ENCOUNTER_UNIT)
    static constexpr uint32_t kMaxEncounterSlots = 5;
    // Returns boss unit guid for the given encounter slot (0 if none)
    uint64_t getEncounterUnitGuid(uint32_t slot) const {
        return (slot < kMaxEncounterSlots) ? encounterUnitGuids_[slot] : 0;
    }

    // Raid target markers (MSG_RAID_TARGET_UPDATE)
    // Icon indices 0-7: Star, Circle, Diamond, Triangle, Moon, Square, Cross, Skull
    static constexpr uint32_t kRaidMarkCount = 8;
    // Both read SocialHandler's marks — the state MSG_RAID_TARGET_UPDATE writes.
    // Returns the GUID marked with the given icon (0 = no mark)
    uint64_t getRaidMarkGuid(uint32_t icon) const;
    // Returns the raid mark icon for a given guid (0xFF = no mark)
    uint8_t getEntityRaidMark(uint64_t guid) const;
    // Set or clear a raid mark on a guid (icon 0-7, or 0xFF to clear)
    void setRaidMark(uint64_t guid, uint8_t icon);

    // ---- LFG / Dungeon Finder ----
    // LFG state (aliased from handler_types.hpp)
    using LfgState = game::LfgState;

    // roles bitmask: 0x02=tank, 0x04=healer, 0x08=dps; pass LFGDungeonEntry ID
    void lfgJoin(uint32_t dungeonId, uint8_t roles);
    void lfgLeave();
    void lfgSetRoles(uint8_t roles);
    void lfgAcceptProposal(uint32_t proposalId, bool accept);
    void lfgSetBootVote(bool vote);
    void lfgTeleport(bool toLfgDungeon = true);
    LfgState getLfgState() const;
    bool isLfgQueued() const;
    bool isLfgInDungeon() const;
    uint32_t getLfgDungeonId() const;
    std::string getCurrentLfgDungeonName() const;
    std::string getMapName(uint32_t mapId) const;
    uint32_t getLfgProposalId() const;
    int32_t  getLfgAvgWaitSec() const;
    uint32_t getLfgTimeInQueueMs() const;
    uint32_t getLfgBootVotes() const;
    uint32_t getLfgBootTotal() const;
    uint32_t getLfgBootTimeLeft() const;
    uint32_t getLfgBootNeeded() const;
    const std::string& getLfgBootTargetName() const;
    const std::string& getLfgBootReason() const;

    // Arena team stats (aliased from handler_types.hpp)
    using ArenaTeamStats = game::ArenaTeamStats;
    const std::vector<ArenaTeamStats>& getArenaTeamStats() const;
    void requestArenaTeamRoster(uint32_t teamId);

    // Arena team roster (aliased from handler_types.hpp)
    using ArenaTeamMember = game::ArenaTeamMember;
    using ArenaTeamRoster = game::ArenaTeamRoster;
    // Returns roster for the given teamId, or nullptr if not yet received
    const ArenaTeamRoster* getArenaTeamRoster(uint32_t teamId) const {
        for (const auto& r : arenaTeamRosters_) {
            if (r.teamId == teamId) return &r;
        }
        return nullptr;
    }

    // ---- Loot ----
    void lootTarget(uint64_t guid);
    void lootItem(uint8_t slotIndex);
    void closeLoot();
    void scheduleGameObjectLootOpen(uint64_t guid, float delaySeconds = 0.35f, uint8_t attempts = 1);

    /// True once the object is known to be a fishing school. A click on one
    /// before its query response arrives still schedules a loot open, and by the
    /// time that fires the metadata has usually landed — so the deferred open
    /// re-checks rather than harvesting a school the player never fished.
    bool isFishingHoleGameObject(uint64_t guid) const;
    void clearPendingGameObjectLootOpen(uint64_t guid);
    bool hasPendingGameObjectLootOpen(uint64_t guid) const;
    bool isGatherGameObject(uint64_t guid) const;
    void despawnGameObjectLocally(uint64_t guid);
    /// Remove a creature corpse client-side once it has been looted empty.
    void despawnCreatureLocally(uint64_t guid);
    void activateSpiritHealer(uint64_t npcGuid);
    bool isLootWindowOpen() const;
    const LootResponseData& getCurrentLoot() const;
    void setAutoLoot(bool enabled);
    bool isAutoLoot() const;
    void setAutoSellGrey(bool enabled);
    bool isAutoSellGrey() const;
    void setAutoRepair(bool enabled);
    bool isAutoRepair() const;

    // Master loot candidates (from SMSG_LOOT_MASTER_LIST)
    const std::vector<uint64_t>& getMasterLootCandidates() const;
    bool hasMasterLootCandidates() const;
    void lootMasterGive(uint8_t lootSlot, uint64_t targetGuid);

    // Group loot roll (aliased from handler_types.hpp)
    using LootRollEntry = game::LootRollEntry;
    bool hasPendingLootRoll() const;
    const LootRollEntry& getPendingLootRoll() const;
    void sendLootRoll(uint64_t objectGuid, uint32_t slot, uint8_t rollType);
    // rollType: 0=need, 1=greed, 2=disenchant, 96=pass

    // Equipment Sets (aliased from handler_types.hpp)
    using EquipmentSetInfo = game::EquipmentSetInfo;
    const std::vector<EquipmentSetInfo>& getEquipmentSets() const;
    bool supportsEquipmentSets() const;
    void useEquipmentSet(uint32_t setId);
    void saveEquipmentSet(const std::string& name, const std::string& iconName = "INV_Misc_QuestionMark",
                          uint64_t existingGuid = 0, uint32_t setIndex = 0xFFFFFFFF);
    void deleteEquipmentSet(uint64_t setGuid);

    // NPC Gossip
    void interactWithNpc(uint64_t guid);
    void interactWithGameObject(uint64_t guid);
    uint64_t getHookedFishingBobberGuid() const { return hookedFishingBobberGuid_; }
    void selectGossipOption(uint32_t optionId);
    void selectGossipQuest(uint32_t questId);
    void acceptQuest();
    void declineQuest();
    void closeGossip();
    // Quest-starting items: right-click triggers quest offer dialog via questgiver protocol
    void offerQuestFromItem(uint64_t itemGuid, uint32_t questId);
    uint64_t getBagItemGuid(int bagIndex, int slotIndex) const;
    bool isGossipWindowOpen() const;
    const GossipMessageData& getCurrentGossip() const;
    const std::string& getNpcText(uint32_t textId) const;
    bool isQuestDetailsOpen();
    const QuestDetailsData& getQuestDetails() const;

    // Gossip POI (aliased from handler_types.hpp)
    using GossipPoi = game::GossipPoi;
    const std::vector<GossipPoi>& getGossipPois() const;
    void clearGossipPois() { gossipPois_.clear(); }

    // Quest turn-in
    bool isQuestRequestItemsOpen() const;
    const QuestRequestItemsData& getQuestRequestItems() const;
    void completeQuest();       // Send CMSG_QUESTGIVER_COMPLETE_QUEST
    void closeQuestRequestItems();

    bool isQuestOfferRewardOpen() const;
    const QuestOfferRewardData& getQuestOfferReward() const;
    void chooseQuestReward(uint32_t rewardIndex);  // Send CMSG_QUESTGIVER_CHOOSE_REWARD
    void closeQuestOfferReward();

    // Quest log
    using QuestLogEntry = QuestHandler::QuestLogEntry;
    const std::vector<QuestLogEntry>& getQuestLog() const;
    // Reconcile collect-item quest objectives against current bag contents.
    // Forwards to QuestHandler; called by InventoryHandler after each rebuild.
    void reconcileQuestItemObjectives(const std::unordered_map<uint32_t, uint32_t>& carriedCounts);
    int getMaxQuestLogSlots() const;
    // QuestSort.dbc name for negative ZoneOrSort values (class/profession/seasonal)
    const std::string& getQuestSortName(uint32_t sortId) const;
    int getSelectedQuestLogIndex() const;
    void setSelectedQuestLogIndex(int idx) { selectedQuestLogIndex_ = idx; }
    void abandonQuest(uint32_t questId);
    void shareQuestWithParty(uint32_t questId);  // CMSG_PUSHQUESTTOPARTY
    bool requestQuestQuery(uint32_t questId, bool force = false);
    bool isQuestTracked(uint32_t questId) const { return trackedQuestIds_.count(questId) > 0; }
    void setQuestTracked(uint32_t questId, bool tracked) {
        const bool changed = tracked ? trackedQuestIds_.insert(questId).second
                                     : trackedQuestIds_.erase(questId) > 0;
        if (changed) saveCharacterConfig();
    }
    const std::unordered_set<uint32_t>& getTrackedQuestIds() const;
    bool isQuestShownOnMap(uint32_t questId) const {
        return mapVisibleQuestIds_.count(questId) > 0;
    }
    void setQuestShownOnMap(uint32_t questId, bool shown) {
        const bool changed = shown ? mapVisibleQuestIds_.insert(questId).second
                                   : mapVisibleQuestIds_.erase(questId) > 0;
        if (changed) saveCharacterConfig();
    }
    const std::unordered_set<uint32_t>& getMapVisibleQuestIds() const {
        return mapVisibleQuestIds_;
    }
    bool isQuestQueryPending(uint32_t questId) const {
        return pendingQuestQueryIds_.count(questId) > 0;
    }
    void clearQuestQueryPending(uint32_t questId) { pendingQuestQueryIds_.erase(questId); }
    const std::unordered_map<uint32_t, uint32_t>& getWorldStates() const { return worldStates_; }
    std::optional<uint32_t> getWorldState(uint32_t key) const {
        auto it = worldStates_.find(key);
        if (it == worldStates_.end()) return std::nullopt;
        return it->second;
    }
    uint32_t getWorldStateMapId() const { return worldStateMapId_; }
    uint32_t getWorldStateZoneId() const { return worldStateZoneId_; }
    uint64_t getActiveCritterGuid() const { return activeCritterGuid_; }
    uint32_t getActiveCritterSpellId() const { return activeCritterSpellId_; }
    void setActiveCritter(uint64_t guid, uint32_t spellId) {
        activeCritterGuid_ = guid;
        activeCritterSpellId_ = spellId;
    }
    /// Send the active companion away. No-op when none is out or the expansion
    /// has no opcode for it.
    void dismissCritter();

    // Mirror timers (0=fatigue, 1=breath, 2=feigndeath)
    struct MirrorTimer {
        int32_t value    = 0;
        int32_t maxValue = 0;
        int32_t scale    = 0;     // +1 = counting up, -1 = counting down
        bool    paused   = false;
        bool    active   = false;
        float   pendingMs = 0.0f; // sub-millisecond carry for the local countdown
    };
    const MirrorTimer& getMirrorTimer(int type) const {
        static MirrorTimer empty;
        return (type >= 0 && type < 3) ? mirrorTimers_[type] : empty;
    }
    /**
     * Count the mirror timers (breath, fatigue, feign death) down locally.
     *
     * The server sends SMSG_START_MIRROR_TIMER once with the remaining time and a
     * scale, then only speaks again when something changes — so without this the
     * breath bar just sits at whatever value it was handed and never moves.
     * scale is ms of timer per ms of real time: -1 while drowning, +1 while the
     * bar refills at the surface.
     */
    void tickMirrorTimers(float dt) {
        if (dt <= 0.0f) return;
        const float elapsedMs = dt * 1000.0f;
        for (auto& t : mirrorTimers_) {
            if (!t.active || t.paused || t.scale == 0 || t.maxValue <= 0) continue;
            // Carry the sub-millisecond remainder: truncating each frame would lose
            // most of a 144fps frame's 6.94ms and run the timer visibly slow.
            t.pendingMs += elapsedMs;
            const int32_t wholeMs = static_cast<int32_t>(t.pendingMs);
            if (wholeMs == 0) continue;
            t.pendingMs -= static_cast<float>(wholeMs);
            t.value = std::clamp(t.value + t.scale * wholeMs, 0, t.maxValue);
        }
    }

    // Combo points
    uint8_t  getComboPoints() const { return comboPoints_; }
    uint8_t  getShapeshiftFormId() const { return shapeshiftFormId_; }
    uint64_t getComboTarget() const { return comboTarget_; }

    // Death Knight rune state (6 runes: 0-1=Blood, 2-3=Unholy, 4-5=Frost; may become Death=3)
    enum class RuneType : uint8_t { Blood = 0, Unholy = 1, Frost = 2, Death = 3 };
    struct RuneSlot {
        RuneType type = RuneType::Blood;
        bool     ready = true;          // Server-confirmed ready state
        float    readyFraction = 1.0f;  // 0.0=depleted → 1.0=full (from server sync)
    };
    const std::array<RuneSlot, 6>& getPlayerRunes() const { return playerRunes_; }

    // Talent-driven spell modifiers (SMSG_SET_FLAT_SPELL_MODIFIER / SMSG_SET_PCT_SPELL_MODIFIER)
    // SpellModOp matches WotLK SpellModOp enum (server-side).
    enum class SpellModOp : uint8_t {
        Damage            =  0,
        Duration          =  1,
        Threat            =  2,
        Effect1           =  3,
        Charges           =  4,
        Range             =  5,
        Radius            =  6,
        CritChance        =  7,
        AllEffects        =  8,
        NotLoseCastingTime =  9,
        CastingTime       = 10,
        Cooldown          = 11,
        Effect2           = 12,
        IgnoreArmor       = 13,
        Cost              = 14,
        CritDamageBonus   = 15,
        ResistMissChance  = 16,
        JumpTargets       = 17,
        ChanceOfSuccess   = 18,
        ActivationTime    = 19,
        Efficiency        = 20,
        MultipleValue     = 21,
        ResistDispelChance = 22,
        Effect3           = 23,
        BonusMultiplier   = 24,
        ProcPerMinute     = 25,
        ValueMultiplier   = 26,
        ResistPushback    = 27,
        MechanicDuration  = 28,
        StartCooldown     = 29,
        PeriodicBonus     = 30,
        AttackPower       = 31,
    };
    static constexpr int SPELL_MOD_OP_COUNT = 32;

    // Key: (SpellModOp, groupIndex) — value: accumulated flat or pct modifier
    // pct values are stored in integer percent (e.g. -20 means -20% reduction).
    struct SpellModKey {
        SpellModOp op;
        uint8_t    group;
        bool operator==(const SpellModKey& o) const {
            return op == o.op && group == o.group;
        }
    };
    struct SpellModKeyHash {
        std::size_t operator()(const SpellModKey& k) const {
            return std::hash<uint32_t>()(
                (static_cast<uint32_t>(static_cast<uint8_t>(k.op)) << 8) | k.group);
        }
    };

    // Returns the sum of all flat modifiers for a given op across all groups.
    // (Callers that need per-group resolution can use getSpellFlatMods() directly.)
    int32_t getSpellFlatMod(SpellModOp op) const {
        int32_t total = 0;
        for (const auto& [k, v] : spellFlatMods_)
            if (k.op == op) total += v;
        return total;
    }
    // Returns the sum of all pct modifiers for a given op across all groups (in %).
    int32_t getSpellPctMod(SpellModOp op) const {
        int32_t total = 0;
        for (const auto& [k, v] : spellPctMods_)
            if (k.op == op) total += v;
        return total;
    }

    // Convenience: apply flat+pct modifier to a base value.
    // result = (base + flatMod) * (1.0 + pctMod/100.0), clamped to >= 0.
    static int32_t applySpellMod(int32_t base, int32_t flat, int32_t pct) {
        int64_t v = static_cast<int64_t>(base) + flat;
        if (pct != 0) v = v + (v * pct + 50) / 100;  // round half-up
        return static_cast<int32_t>(v < 0 ? 0 : v);
    }

    struct FactionStandingInit {
        uint8_t flags = 0;
        int32_t standing = 0;
    };
    // Faction flag bitmask constants (from Faction.dbc ReputationFlags / SMSG_INITIALIZE_FACTIONS)
    static constexpr uint8_t FACTION_FLAG_VISIBLE    = 0x01; // shown in reputation list
    static constexpr uint8_t FACTION_FLAG_AT_WAR     = 0x02; // player is at war
    static constexpr uint8_t FACTION_FLAG_HIDDEN      = 0x04; // never shown
    static constexpr uint8_t FACTION_FLAG_INVISIBLE_FORCED = 0x08;
    static constexpr uint8_t FACTION_FLAG_PEACE_FORCED     = 0x10;
    static constexpr uint8_t FACTION_FLAG_INACTIVE         = 0x20; // moved to the inactive list

    const std::vector<FactionStandingInit>& getInitialFactions() const { return initialFactions_; }
    const std::unordered_map<uint32_t, int32_t>& getFactionStandings() const { return factionStandings_; }

    // Returns true if the player has "at war" toggled for the faction at repListId
    bool isFactionAtWar(uint32_t repListId) const {
        if (repListId >= initialFactions_.size()) return false;
        return (initialFactions_[repListId].flags & FACTION_FLAG_AT_WAR) != 0;
    }
    // Returns true if the faction is visible in the reputation list
    bool isFactionVisible(uint32_t repListId) const {
        if (repListId >= initialFactions_.size()) return false;
        const uint8_t f = initialFactions_[repListId].flags;
        if (f & FACTION_FLAG_HIDDEN) return false;
        if (f & FACTION_FLAG_INVISIBLE_FORCED) return false;
        return (f & FACTION_FLAG_VISIBLE) != 0;
    }
    // Returns true if the faction has been set inactive (hidden from the active list)
    bool isFactionInactive(uint32_t repListId) const {
        if (repListId >= initialFactions_.size()) return false;
        return (initialFactions_[repListId].flags & FACTION_FLAG_INACTIVE) != 0;
    }
    // Returns true if war cannot be declared on this faction (peace forced by the server)
    bool isFactionPeaceForced(uint32_t repListId) const {
        if (repListId >= initialFactions_.size()) return false;
        return (initialFactions_[repListId].flags & FACTION_FLAG_PEACE_FORCED) != 0;
    }
    // Returns the faction ID for a given repListId (0 if unknown)
    uint32_t getFactionIdByRepListId(uint32_t repListId) const;
    // Returns the repListId for a given faction ID (0xFFFFFFFF if not found)
    uint32_t getRepListIdByFactionId(uint32_t factionId) const;
    // Shaman totems (4 slots: 0=Earth, 1=Fire, 2=Water, 3=Air)
    struct TotemSlot {
        uint32_t spellId     = 0;
        uint32_t durationMs  = 0;
        std::chrono::steady_clock::time_point placedAt{};
        bool active() const { return spellId != 0 && remainingMs() > 0; }
        float remainingMs() const {
            if (spellId == 0 || durationMs == 0) return 0.0f;
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - placedAt).count();
            float rem = static_cast<float>(durationMs) - static_cast<float>(elapsed);
            return rem > 0.0f ? rem : 0.0f;
        }
    };
    static constexpr int NUM_TOTEM_SLOTS = 4;
    const TotemSlot& getTotemSlot(int slot) const {
        static TotemSlot empty;
        return (slot >= 0 && slot < NUM_TOTEM_SLOTS) ? activeTotemSlots_[slot] : empty;
    }

    const std::string& getFactionNamePublic(uint32_t factionId) const;
    uint32_t getWatchedFactionId() const { return watchedFactionId_; }
    void setWatchedFactionId(uint32_t factionId);
    // Declare war / make peace with a faction (CMSG_SET_FACTION_ATWAR). No-op on
    // peace-forced factions. Updates the local flag optimistically.
    void setFactionAtWar(uint32_t repListId, bool atWar);
    // Move a faction to / from the inactive list (CMSG_SET_FACTION_INACTIVE).
    void setFactionInactive(uint32_t repListId, bool inactive);
    uint32_t getLastContactListMask() const { return lastContactListMask_; }
    uint32_t getLastContactListCount() const { return lastContactListCount_; }
    bool isServerMovementAllowed() const;

    // Quest giver status (! and ? markers)
    QuestGiverStatus getQuestGiverStatus(uint64_t guid) const;
    const std::unordered_map<uint64_t, QuestGiverStatus>& getNpcQuestStatuses() const;

    // Charge callback — fires when player casts a charge spell toward target
    // Parameters: targetGuid, targetX, targetY, targetZ (canonical WoW coordinates)
    using ChargeCallback = std::function<void(uint64_t targetGuid, float x, float y, float z)>;
    void setChargeCallback(ChargeCallback cb) { chargeCallback_ = std::move(cb); }

    // Level-up callback — fires when the player gains a level (newLevel > 1)
    using LevelUpCallback = std::function<void(uint32_t newLevel)>;
    void setLevelUpCallback(LevelUpCallback cb) { levelUpCallback_ = std::move(cb); }

    // Stat deltas from the last SMSG_LEVELUP_INFO (valid until next level-up)
    struct LevelUpDeltas {
        uint32_t hp   = 0;
        uint32_t mana = 0;
        uint32_t str = 0, agi = 0, sta = 0, intel = 0, spi = 0;
    };
    const LevelUpDeltas& getLastLevelUpDeltas() const { return lastLevelUpDeltas_; }

    // Temporary weapon enchant timers (from SMSG_ITEM_ENCHANT_TIME_UPDATE)
    // Slot: 0=main-hand, 1=off-hand, 2=ranged. Value: expire time (steady_clock ms).
    struct TempEnchantTimer {
        uint32_t slot     = 0;
        uint64_t expireMs = 0;   // std::chrono::steady_clock ms timestamp when it expires
    };
    const std::vector<TempEnchantTimer>& getTempEnchantTimers() const { return tempEnchantTimers_; }
    // Returns remaining ms for a given slot, or 0 if absent/expired.
    uint32_t getTempEnchantRemainingMs(uint32_t slot) const;
    static constexpr const char* kTempEnchantSlotNames[] = { "Main Hand", "Off Hand", "Ranged" };

    // ---- Readable text (books / scrolls / notes) ----
    // Populated by handlePageTextQueryResponse(); multi-page items chain via nextPageId.
    struct BookPage { uint32_t pageId = 0; std::string text; };
    const std::vector<BookPage>& getBookPages() const { return bookPages_; }
    bool hasBookOpen() const { return !bookPages_.empty(); }
    void clearBook() { bookPages_.clear(); }

    // Other player level-up callback — fires when another player gains a level
    using OtherPlayerLevelUpCallback = std::function<void(uint64_t guid, uint32_t newLevel)>;
    void setOtherPlayerLevelUpCallback(OtherPlayerLevelUpCallback cb) { otherPlayerLevelUpCallback_ = std::move(cb); }

    // Achievement earned callback — fires when SMSG_ACHIEVEMENT_EARNED is received
    using AchievementEarnedCallback = std::function<void(uint32_t achievementId, const std::string& name)>;
    void setAchievementEarnedCallback(AchievementEarnedCallback cb) { achievementEarnedCallback_ = std::move(cb); }
    const std::unordered_set<uint32_t>& getEarnedAchievements() const { return earnedAchievements_; }

    // Title system — earned title bits and the currently displayed title
    const std::unordered_set<uint32_t>& getKnownTitleBits() const { return knownTitleBits_; }
    int32_t getChosenTitleBit() const { return chosenTitleBit_; }
    /// Returns the formatted title string for a given bit (replaces %s with player name), or empty.
    std::string getFormattedTitle(uint32_t bit) const;
    /// Send CMSG_SET_TITLE to activate a title (bit >= 0) or clear it (bit = -1).
    void sendSetTitle(int32_t bit);

    // Area discovery callback — fires when SMSG_EXPLORATION_EXPERIENCE is received
    using AreaDiscoveryCallback = std::function<void(const std::string& areaName, uint32_t xpGained)>;
    void setAreaDiscoveryCallback(AreaDiscoveryCallback cb) { areaDiscoveryCallback_ = std::move(cb); }

    // Quest objective progress callback — fires on SMSG_QUESTUPDATE_ADD_KILL / ADD_ITEM
    // questTitle: name of the quest; objectiveName: creature/item name; current/required counts
    using QuestProgressCallback = std::function<void(const std::string& questTitle,
                                                     const std::string& objectiveName,
                                                     uint32_t current, uint32_t required)>;
    void setQuestProgressCallback(QuestProgressCallback cb) { questProgressCallback_ = std::move(cb); }
    const std::unordered_map<uint32_t, uint64_t>& getCriteriaProgress() const { return criteriaProgress_; }
    /// Returns the WoW PackedTime earn date for an achievement, or 0 if unknown.
    uint32_t getAchievementDate(uint32_t id) const {
        auto it = achievementDates_.find(id);
        return (it != achievementDates_.end()) ? it->second : 0u;
    }
    /// Returns the name of an achievement by ID, or empty string if unknown.
    const std::string& getAchievementName(uint32_t id) const {
        auto it = achievementNameCache_.find(id);
        if (it != achievementNameCache_.end()) return it->second;
        static const std::string kEmpty;
        return kEmpty;
    }
    void ensureAchievementNamesLoaded() { loadAchievementNameCache(); }
    /// Returns the description of an achievement by ID, or empty string if unknown.
    const std::string& getAchievementDescription(uint32_t id) const {
        auto it = achievementDescCache_.find(id);
        if (it != achievementDescCache_.end()) return it->second;
        static const std::string kEmpty;
        return kEmpty;
    }
    /// Returns the point value of an achievement by ID, or 0 if unknown.
    uint32_t getAchievementPoints(uint32_t id) const {
        auto it = achievementPointsCache_.find(id);
        return (it != achievementPointsCache_.end()) ? it->second : 0u;
    }
    /// Returns the SpellIcon.dbc ID for an achievement's icon, or 0 if unknown.
    uint32_t getAchievementIconId(uint32_t id) const {
        auto it = achievementIconCache_.find(id);
        return (it != achievementIconCache_.end()) ? it->second : 0u;
    }
    /// Returns the set of achievement IDs earned by an inspected player (via SMSG_RESPOND_INSPECT_ACHIEVEMENTS).
    /// Returns nullptr if no inspect data is available for the given GUID.
    const std::unordered_set<uint32_t>* getInspectedPlayerAchievements(uint64_t guid) const {
        auto it = inspectedPlayerAchievements_.find(guid);
        return (it != inspectedPlayerAchievements_.end()) ? &it->second : nullptr;
    }

    // Server-triggered music callback — fires when SMSG_PLAY_MUSIC is received.
    // The soundId corresponds to a SoundEntries.dbc record. The receiver is
    // responsible for looking up the file path and forwarding to MusicManager.
    using PlayMusicCallback = std::function<void(uint32_t soundId)>;
    void setPlayMusicCallback(PlayMusicCallback cb) { playMusicCallback_ = std::move(cb); }

    // Server-triggered 2-D sound effect callback — fires when SMSG_PLAY_SOUND is received.
    // The soundId corresponds to a SoundEntries.dbc record.
    using PlaySoundCallback = std::function<void(uint32_t soundId)>;
    void setPlaySoundCallback(PlaySoundCallback cb) { playSoundCallback_ = std::move(cb); }

    // Server-triggered 3-D positional sound callback — fires for SMSG_PLAY_OBJECT_SOUND and
    // SMSG_PLAY_SPELL_IMPACT. Includes sourceGuid so the receiver can look up world position.
    using PlayPositionalSoundCallback = std::function<void(uint32_t soundId, uint64_t sourceGuid)>;
    void setPlayPositionalSoundCallback(PlayPositionalSoundCallback cb) { playPositionalSoundCallback_ = std::move(cb); }

    // UI error frame: prominent on-screen error messages (spell can't be cast, etc.)
    using UIErrorCallback = std::function<void(const std::string& msg)>;
    void setUIErrorCallback(UIErrorCallback cb) { uiErrorCallback_ = std::move(cb); }
    void addUIError(const std::string& msg) {
        if (uiErrorCallback_) uiErrorCallback_(msg);
        fireAddonEvent("UI_ERROR_MESSAGE", {msg});
    }
    /// A script that failed, shown on screen without telling any script about it.
    ///
    /// Deliberately not addUIError. That fires UI_ERROR_MESSAGE, which is a Lua
    /// event, and UIErrorsFrame is registered for it — so reporting a script
    /// error ran script, and if that script errored it was reported the same
    /// way. One broken handler fed itself as fast as the frame loop allowed
    /// until the process died, which is not what a Lua error looks like from
    /// outside and cost a long hunt through the renderer.
    ///
    /// UI_ERROR_MESSAGE is also the wrong event for this: it carries the game's
    /// own refusals — out of range, not enough mana — and the real client
    /// reports script errors somewhere else entirely.
    void addScriptError(const std::string& msg) {
        if (uiErrorCallback_) uiErrorCallback_(msg);
    }
    void addUIInfoMessage(const std::string& msg) {
        fireAddonEvent("UI_INFO_MESSAGE", {msg});
    }
    void fireAddonEvent(const std::string& event, const std::vector<std::string>& args = {}) {
        if (addonEventCallback_) addonEventCallback_(event, args);
    }
    // Convenience: invoke a callback with a sound manager obtained from the renderer.
    template<typename ManagerGetter, typename Callback>
    void withSoundManager(ManagerGetter getter, Callback cb) {
        if (auto* ac = services_.audioCoordinator) {
            if (auto* mgr = (ac->*getter)()) cb(mgr);
        }
    }
    // Play the player character's spoken error response ("Not enough mana", ...)
    // using the active character's race/gender. Gated by the Character Speech setting.
    void playErrorSpeech(audio::PlayerErrorSpeech type);

    // Reputation change toast: factionName, delta, new standing
    using RepChangeCallback = std::function<void(const std::string& factionName, int32_t delta, int32_t standing)>;
    void setRepChangeCallback(RepChangeCallback cb) { repChangeCallback_ = std::move(cb); }

    // PvP honor credit callback (honorable kill or BG reward)
    using PvpHonorCallback = std::function<void(uint32_t honorAmount, uint64_t victimGuid, uint32_t victimRank)>;
    void setPvpHonorCallback(PvpHonorCallback cb) { pvpHonorCallback_ = std::move(cb); }

    // Item looted / received callback (SMSG_ITEM_PUSH_RESULT when showInChat is set)
    using ItemLootCallback = std::function<void(uint32_t itemId, uint32_t count, uint32_t quality, const std::string& name)>;
    void setItemLootCallback(ItemLootCallback cb) { itemLootCallback_ = std::move(cb); }

    // Loot window open/close callback (for loot kneel animation)
    using LootWindowCallback = std::function<void(bool open)>;
    void setLootWindowCallback(LootWindowCallback cb) { lootWindowCallback_ = std::move(cb); }

    // Quest turn-in completion callback
    using QuestCompleteCallback = std::function<void(uint32_t questId, const std::string& questTitle)>;
    void setQuestCompleteCallback(QuestCompleteCallback cb) { questCompleteCallback_ = std::move(cb); }

    // Mount state
    using MountCallback = std::function<void(uint32_t mountDisplayId)>;  // 0 = dismount
    void setMountCallback(MountCallback cb) { mountCallback_ = std::move(cb); }

    // Mount display changes for visible players other than the local character.
    using OtherPlayerMountCallback = std::function<void(uint64_t guid, uint32_t mountDisplayId)>;
    void setOtherPlayerMountCallback(OtherPlayerMountCallback cb) { otherPlayerMountCallback_ = std::move(cb); }

    // Taxi terrain precaching callback
    using TaxiPrecacheCallback = std::function<void(const std::vector<glm::vec3>&)>;
    void setTaxiPrecacheCallback(TaxiPrecacheCallback cb) { taxiPrecacheCallback_ = std::move(cb); }

    // Taxi orientation callback (for mount rotation: yaw, pitch, roll in radians)
    using TaxiOrientationCallback = std::function<void(float yaw, float pitch, float roll)>;
    void setTaxiOrientationCallback(TaxiOrientationCallback cb) { taxiOrientationCallback_ = std::move(cb); }

    // Taxi landing position correction callback (canonical x, y, z). Application's
    // per-frame render-position sync only pulls from game state while onTaxi (see
    // its "Sync character render position" block) - once finishClientTaxiFlight()
    // clears onTaxiFlight_/taxiMountActive_, that sync stops, so a position
    // correction applied only to movementInfo/the entity (e.g. snapping to the
    // known-correct TaxiNodes.dbc position instead of a short-landed spline) never
    // reaches the renderer, which then stays authoritative at its last (wrong)
    // value. Live-confirmed: the landing clamp reads renderer->getCharacterPosition()
    // directly and kept computing from the pre-correction position even after
    // finishClientTaxiFlight() had already fixed movementInfo/the entity. This
    // callback lets MovementHandler push the corrected position straight to the
    // renderer at the same moment, without MovementHandler needing direct access
    // to rendering types.
    using TaxiLandingPositionCallback = std::function<void(float x, float y, float z)>;
    void setTaxiLandingPositionCallback(TaxiLandingPositionCallback cb) { taxiLandingPositionCallback_ = std::move(cb); }

    // Callback for when taxi flight is about to start (after mounting delay, before movement begins)
    using TaxiFlightStartCallback = std::function<void()>;
    void setTaxiFlightStartCallback(TaxiFlightStartCallback cb) { taxiFlightStartCallback_ = std::move(cb); }

    // Callback fired when server sends SMSG_OPEN_LFG_DUNGEON_FINDER (open dungeon finder UI)
    using OpenLfgCallback = std::function<void()>;
    void setOpenLfgCallback(OpenLfgCallback cb) { openLfgCallback_ = std::move(cb); }

    bool isMounted() const { return currentMountDisplayId_ != 0; }
    bool isHostileAttacker(uint64_t guid) const;
    bool isHostileFactionPublic(uint32_t factionTemplateId) const { return isHostileFaction(factionTemplateId); }
    float getServerRunSpeed() const;
    float getServerWalkSpeed() const;
    float getServerSwimSpeed() const;
    float getServerSwimBackSpeed() const;
    float getServerFlightSpeed() const;
    float getServerFlightBackSpeed() const;
    float getServerRunBackSpeed() const;
    float getServerTurnRate() const;
    bool isPlayerRooted() const {
        return (movementInfo.flags & static_cast<uint32_t>(MovementFlags::ROOT)) != 0;
    }
    bool isGravityDisabled() const {
        return (movementInfo.flags & static_cast<uint32_t>(MovementFlags::LEVITATING)) != 0;
    }
    bool isFeatherFalling() const {
        return (movementInfo.flags & static_cast<uint32_t>(MovementFlags::FEATHER_FALL)) != 0;
    }
    bool isWaterWalking() const {
        return (movementInfo.flags & static_cast<uint32_t>(MovementFlags::WATER_WALK)) != 0;
    }
    bool isPlayerFlying() const {
        const uint32_t flyMask = static_cast<uint32_t>(MovementFlags::CAN_FLY) |
                                 static_cast<uint32_t>(MovementFlags::FLYING);
        return (movementInfo.flags & flyMask) == flyMask;
    }
    // The player is *allowed* to fly (CAN_FLY set by a flying mount or .gm fly).
    // Drives flight-mode physics: FLYING is only set once actually airborne, so
    // gating flight on isPlayerFlying() left .gm fly unable to take off.
    bool canFly() const {
        return (movementInfo.flags & static_cast<uint32_t>(MovementFlags::CAN_FLY)) != 0;
    }
    bool isHovering() const {
        return (movementInfo.flags & static_cast<uint32_t>(MovementFlags::HOVER)) != 0;
    }
    bool isSwimming() const {
        return (movementInfo.flags & static_cast<uint32_t>(MovementFlags::SWIMMING)) != 0;
    }
    // Set the character pitch angle (radians) for movement packets (flight / swimming).
    // Positive = nose up, negative = nose down.
    void setMovementPitch(float radians) { movementInfo.pitch = radians; }
    void dismount();

    // Taxi / Flight Paths
    bool isTaxiWindowOpen() const;
    void closeTaxi();
    void activateTaxi(uint32_t destNodeId);
    bool isOnTaxiFlight() const;
    bool isTaxiMountActive() const;
    bool isTaxiActivationPending() const;
    void forceClearTaxiAndMovementState();
    const std::string& getTaxiDestName() const;
    const ShowTaxiNodesData& getTaxiData() const;
    uint32_t getTaxiCurrentNode() const;

    using TaxiNode = MovementHandler::TaxiNode;
    using TaxiPathEdge = MovementHandler::TaxiPathEdge;
    using TaxiPathNode = MovementHandler::TaxiPathNode;
    const std::unordered_map<uint32_t, TaxiNode>& getTaxiNodes() const;
    bool isKnownTaxiNode(uint32_t nodeId) const;
    uint32_t getTaxiCostTo(uint32_t destNodeId) const;
    bool hasTaxiRouteTo(uint32_t destNodeId) const;
    std::vector<uint32_t> getTaxiRouteTo(uint32_t destNodeId) const;
    bool taxiNpcHasRoutes(uint64_t guid) const {
        auto it = taxiNpcHasRoutes_.find(guid);
        return it != taxiNpcHasRoutes_.end() && it->second;
    }

    // Vehicle (WotLK)
    bool isInVehicle() const { return vehicleId_ != 0; }
    uint32_t getVehicleId() const { return vehicleId_; }
    void sendRequestVehicleExit();

    // Vendor
    void openVendor(uint64_t npcGuid);
    void closeVendor();
    void buyItem(uint64_t vendorGuid, uint32_t itemId, uint32_t slot, uint32_t count);
    void sellItem(uint64_t vendorGuid, uint64_t itemGuid, uint32_t count);
    void sellItemBySlot(int backpackIndex);
    void sellItemInBag(int bagIndex, int slotIndex);
    struct BuybackItem {
        uint64_t itemGuid = 0;
        ItemDef item;
        uint32_t count = 1;
        uint32_t wireSlot = 0;
    };
    void buyBackItem(uint32_t buybackSlot);
    void repairItem(uint64_t vendorGuid, uint64_t itemGuid);
    void repairAll(uint64_t vendorGuid, bool useGuildBank = false);
    uint32_t estimateRepairAllCost() const;
    const std::deque<BuybackItem>& getBuybackItems() const;
    void autoEquipItemBySlot(int backpackIndex);
    void autoEquipItemInBag(int bagIndex, int slotIndex);
    void useItemBySlot(int backpackIndex);
    void useItemInBag(int bagIndex, int slotIndex);

    // Item-targeted item use: sharpening stones, weightstones and weapon oils enchant
    // another item, so using one arms a targeting cursor instead of casting immediately.
    bool isAwaitingItemTarget() const;
    /// Arm the item-picking cursor for a spell that must be cast at an item.
    void beginSpellItemTargeting(uint32_t spellId, const std::string& spellName);
    uint32_t getPendingItemTargetSourceItemId() const;
    void cancelItemTargeting();
    void completeItemUseOnItem(uint64_t targetItemGuid);

    // CMSG_OPEN_ITEM — for locked containers (lockboxes); server checks keyring automatically
    void openItemBySlot(int backpackIndex);
    void openItemInBag(int bagIndex, int slotIndex);
    void readItemBySlot(int backpackIndex);
    void readItemInBag(int bagIndex, int slotIndex);
    void destroyItem(uint8_t bag, uint8_t slot, uint8_t count = 1);
    void splitItem(uint8_t srcBag, uint8_t srcSlot, uint8_t count);
    void swapContainerItems(uint8_t srcBag, uint8_t srcSlot, uint8_t dstBag, uint8_t dstSlot);
    void swapBagSlots(int srcBagIndex, int dstBagIndex);
    void useItemById(uint32_t itemId);
    uint32_t getItemIdForSpell(uint32_t spellId) const;
    bool isVendorWindowOpen() const;
    const ListInventoryData& getVendorItems() const;
    void setVendorCanRepair(bool v);

    // Mail
    bool isMailboxOpen() const;
    const std::vector<MailMessage>& getMailInbox() const;
    std::string getMailDisplaySubject(const MailMessage& mail);
    int getSelectedMailIndex() const;
    void setSelectedMailIndex(int idx);
    bool isMailComposeOpen() const;
    void openMailCompose();
    void closeMailCompose();
    bool hasNewMail() const;
    void openMailbox(uint64_t guid);
    void closeMailbox();
    void sendMail(const std::string& recipient, const std::string& subject,
                  const std::string& body, uint64_t money, uint64_t cod = 0);

    // Mail attachments (max 12 per WotLK)
    static constexpr int MAIL_MAX_ATTACHMENTS = 12;
    struct MailAttachSlot {
        uint64_t itemGuid = 0;
        game::ItemDef item;
        uint8_t srcBag = 0xFF;   // source container for return
        uint8_t srcSlot = 0;
        bool occupied() const { return itemGuid != 0; }
    };
    bool attachItemFromBackpack(int backpackIndex);
    bool attachItemFromBag(int bagIndex, int slotIndex);
    bool detachMailAttachment(int attachIndex);
    void clearMailAttachments();
    const std::array<MailAttachSlot, 12>& getMailAttachments() const;
    int getMailAttachmentCount() const;
    /// Attachments this realm's mail packet can actually carry — one on Vanilla,
    /// twelve from TBC on. The compose window used to offer twelve regardless
    /// and quietly send the first.
    int getMaxMailAttachments() const;
    void mailTakeMoney(uint32_t mailId);
    void mailTakeItem(uint32_t mailId, uint32_t itemGuidLow);
    void mailDelete(uint32_t mailId);
    void mailMarkAsRead(uint32_t mailId);
    void refreshMailList();

    // Bank
    void openBank(uint64_t guid);
    void closeBank();
    void buyBankSlot();
    // Copper cost of the bank bag slot at the given 0-based index (BankBagSlotPrices.dbc).
    uint32_t getBankBagSlotPrice(int slotIndex) const;
    void depositItem(uint8_t srcBag, uint8_t srcSlot);
    void withdrawItem(uint8_t srcBag, uint8_t srcSlot);
    bool isBankOpen() const;
    uint64_t getBankerGuid() const;
    int getEffectiveBankSlots() const;
    int getEffectiveBankBagSlots() const;

    // Guild Bank
    void openGuildBank(uint64_t guid);
    void closeGuildBank();
    void queryGuildBankTab(uint8_t tabId);
    void buyGuildBankTab();
    void depositGuildBankMoney(uint32_t amount);
    void withdrawGuildBankMoney(uint32_t amount);
    void guildBankWithdrawItem(uint8_t tabId, uint8_t bankSlot, uint8_t destBag, uint8_t destSlot);
    void guildBankDepositItem(uint8_t tabId, uint8_t bankSlot, uint8_t srcBag, uint8_t srcSlot);
    void guildBankDepositFromInventory(uint8_t srcBag, uint8_t srcSlot);
    bool isGuildBankOpen() const;
    const GuildBankData& getGuildBankData() const;
    uint8_t getGuildBankActiveTab() const;
    void setGuildBankActiveTab(uint8_t tab);

    // Auction House
    void openAuctionHouse(uint64_t guid);
    void closeAuctionHouse();
    void auctionSearch(const std::string& name, uint8_t levelMin, uint8_t levelMax,
                       uint32_t quality, uint32_t itemClass, uint32_t itemSubClass,
                       uint32_t invTypeMask, uint8_t usableOnly, uint32_t offset = 0);
    void auctionSellItem(int backpackIndex, uint32_t bid,
                         uint32_t buyout, uint32_t duration);
    void auctionSellItemByGuid(uint64_t itemGuid, uint32_t stackCount, uint32_t bid,
                               uint32_t buyout, uint32_t duration);
    void auctionPlaceBid(uint32_t auctionId, uint32_t amount);
    void auctionBuyout(uint32_t auctionId, uint32_t buyoutPrice);
    void auctionCancelItem(uint32_t auctionId);
    void auctionListOwnerItems(uint32_t offset = 0);
    void auctionListBidderItems(uint32_t offset = 0);
    bool isAuctionHouseOpen() const;
    uint64_t getAuctioneerGuid() const;
    const AuctionListResult& getAuctionBrowseResults() const;
    const AuctionListResult& getAuctionOwnerResults() const;
    const AuctionListResult& getAuctionBidderResults() const;
    int getAuctionActiveTab() const;
    void setAuctionActiveTab(int tab);
    float getAuctionSearchDelay() const;

    // Trainer
    bool isTrainerWindowOpen() const;
    const TrainerListData& getTrainerSpells() const;
    void trainSpell(uint32_t spellId);
    void closeTrainer();
    const std::string& getSpellName(uint32_t spellId) const;
    const std::string& getSpellRank(uint32_t spellId) const;
    /// Returns the tooltip/description text from Spell.dbc (empty if unknown or has no text).
    const std::string& getSpellDescription(uint32_t spellId) const;
    /// Substitute WoW description tokens in `raw` using live spell data: $s/$o/$m/$M base
    /// points and $d durations (incl. cross-spell $<spellId> references), plus $l/$g
    /// plural/gender forms. Unresolvable tokens ($h proc chance, $t period) are stripped
    /// cleanly. `selfSpellId` supplies the default source for index-only tokens like $s1.
    std::string formatSpellDescription(uint32_t selfSpellId, const std::string& raw) const;
    // SpellFocusObject.dbc name ("Anvil", "Cooking Fire", ...) for
    // requires-spell-focus cast failures; empty if unknown.
    const std::string& getSpellFocusName(uint32_t focusId) const;
    // TotemCategory.dbc name ("Blacksmith Hammer", "Mining Pick", ...) for
    // totem-category cast failures; empty if unknown.
    const std::string& getTotemCategoryName(uint32_t categoryId) const;
    const int32_t* getSpellEffectBasePoints(uint32_t spellId) const;
    float getSpellDuration(uint32_t spellId) const;
    std::string getEnchantName(uint32_t enchantId) const;
    const std::string& getSkillLineName(uint32_t spellId) const;
    /// Returns the DispelType for a spell (0=none,1=magic,2=curse,3=disease,4=poison,5+=other)
    uint8_t getSpellDispelType(uint32_t spellId) const;
    /// Returns true if the spell can be interrupted by abilities like Kick/Counterspell.
    /// False for spells with SPELL_ATTR_EX_NOT_INTERRUPTIBLE (attrEx bit 4 = 0x10).
    bool isSpellInterruptible(uint32_t spellId) const;
    /// Returns the school bitmask for the spell from Spell.dbc
    /// (0x01=Physical, 0x02=Holy, 0x04=Fire, 0x08=Nature, 0x10=Frost, 0x20=Shadow, 0x40=Arcane).
    /// Returns 0 if unknown.
    uint32_t getSpellSchoolMask(uint32_t spellId) const;
    /// Returns the Spell.dbc Targets bitmask (SpellCastTargetFlags) for the spell.
    /// 0x10 = TARGET_FLAG_ITEM, meaning the spell must be cast onto another item.
    uint32_t getSpellTargetFlags(uint32_t spellId) const;
    // Spell.dbc TargetAuraState (aura state the target must be in; 0 = no requirement).
    uint32_t getSpellTargetAuraState(uint32_t spellId) const;

    struct TrainerTab {
        std::string name;
        std::vector<const TrainerSpell*> spells;
    };
    const std::vector<TrainerTab>& getTrainerTabs() const;
    const ItemQueryResponseData* getItemInfo(uint32_t itemId) const {
        auto it = itemInfoCache_.find(itemId);
        return (it != itemInfoCache_.end()) ? &it->second : nullptr;
    }
    const std::unordered_map<uint32_t, ItemQueryResponseData>& getItemInfoCache() const { return itemInfoCache_; }
    // Request item info from server if not already cached/pending
    void ensureItemInfo(uint32_t entry) {
        if (entry == 0 || itemInfoCache_.count(entry) || pendingItemQueries_.count(entry)) return;
        queryItemInfo(entry, 0);
    }
    uint64_t getBackpackItemGuid(int index) const {
        if (index < 0 || index >= static_cast<int>(backpackSlotGuids_.size())) return 0;
        return backpackSlotGuids_[index];
    }
    uint64_t getEquipSlotGuid(int slot) const {
        if (slot < 0 || slot >= static_cast<int>(equipSlotGuids_.size())) return 0;
        return equipSlotGuids_[slot];
    }
    // Returns the permanent and temporary enchant IDs for an item by GUID (0 if unknown).
    std::pair<uint32_t, uint32_t> getItemEnchantIds(uint64_t guid) const {
        auto it = onlineItems_.find(guid);
        if (it == onlineItems_.end()) return {0, 0};
        return {it->second.permanentEnchantId, it->second.temporaryEnchantId};
    }
    // Returns the socket gem enchant IDs (3 slots; 0 = empty socket) for an item by GUID.
    std::array<uint32_t, 3> getItemSocketEnchantIds(uint64_t guid) const {
        auto it = onlineItems_.find(guid);
        if (it == onlineItems_.end()) return {};
        return it->second.socketEnchantIds;
    }
    uint64_t getVendorGuid() const;

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
    void updateNetworking(float deltaTime);
    void updateTimers(float deltaTime);
    void updateEntityInterpolation(float deltaTime);
    void updateTaxiAndMountState(float deltaTime);
    void updateAutoAttack(float deltaTime);

    /**
     * Reset DBC-backed caches so they reload from new expansion data.
     * Called by Application when the expansion profile changes.
     */
    void resetDbcCaches();

    // ═══════════════════════════════════════════════════════════════════
    //  Domain handler access — public accessors for friend-class elimination
    // ═══════════════════════════════════════════════════════════════════

    // ── Handler & Subsystem Accessors (unique_ptr → raw pointer) ─────
    network::WorldSocket* getSocket() { return socket.get(); }
    const network::WorldSocket* getSocket() const { return socket.get(); }
    ChatHandler* getChatHandler() { return chatHandler_.get(); }
    CombatHandler* getCombatHandler() { return combatHandler_.get(); }
    MovementHandler* getMovementHandler() { return movementHandler_.get(); }
    SpellHandler* getSpellHandler() { return spellHandler_.get(); }

    // ── Mutable Accessors for Members with Existing Const Getters ────
    void setTargetGuidRaw(uint64_t g) { targetGuid = g; }
    uint64_t& lastTargetGuidRef() { return lastTargetGuid; }
    uint64_t& focusGuidRef() { return focusGuid; }
    uint64_t& mouseoverGuidRef() { return mouseoverGuid_; }
    MovementInfo& movementInfoRef() { return movementInfo; }
    Inventory& inventoryRef() { return inventory; }

    // ── Core / Session ───────────────────────────────────────────────
    uint32_t getBuild() const { return build; }
    const std::vector<uint8_t>& getSessionKey() const { return sessionKey; }
    auto& charactersRef() { return characters; }
    auto& updateFieldTableRef() { return updateFieldTable_; }
    auto& lastPlayerFieldsRef() { return lastPlayerFields_; }
    auto& timeSinceLastPingRef() { return timeSinceLastPing; }
    auto& activeCharacterGuidRef() { return activeCharacterGuid_; }

    // ── Character & Appearance ───────────────────────────────────────
    auto& chosenTitleBitRef() { return chosenTitleBit_; }
    auto& cloakVisibleRef() { return cloakVisible_; }
    auto& helmVisibleRef() { return helmVisible_; }
    auto& currentMountDisplayIdRef() { return currentMountDisplayId_; }
    auto& mountAuraSpellIdRef() { return mountAuraSpellId_; }
    /// Spell of the aura currently keeping the player mounted, 0 when on foot.
    /// Pressing this one again is a dismount, not a second mount.
    uint32_t getMountAuraSpellId() const { return mountAuraSpellId_; }
    auto& shapeshiftFormIdRef() { return shapeshiftFormId_; }
    auto& playerRaceRef() { return playerRace_; }
    auto& serverPlayerLevelRef() { return serverPlayerLevel_; }

    // ── AFK / DND ────────────────────────────────────────────────────
    auto& afkMessageRef() { return afkMessage_; }
    auto& afkStatusRef() { return afkStatus_; }
    auto& dndMessageRef() { return dndMessage_; }
    auto& dndStatusRef() { return dndStatus_; }

    // ── Movement & Transport ─────────────────────────────────────────
    auto& followRenderPosRef() { return followRenderPos_; }
    auto& followTargetGuidRef() { return followTargetGuid_; }
    auto& onTaxiFlightRef() { return onTaxiFlight_; }
    auto& taxiLandingCooldownRef() { return taxiLandingCooldown_; }
    auto& taxiMountActiveRef() { return taxiMountActive_; }
    auto& taxiStartGraceRef() { return taxiStartGrace_; }
    auto& vehicleIdRef() { return vehicleId_; }
    auto& playerTransportGuidRef() { return playerTransportGuid_; }
    auto& playerTransportOffsetRef() { return playerTransportOffset_; }
    auto& playerTransportStickyGuidRef() { return playerTransportStickyGuid_; }
    auto& playerTransportStickyTimerRef() { return playerTransportStickyTimer_; }
    auto& transportAttachmentsRef() { return transportAttachments_; }

    // ── Inventory & Equipment ────────────────────────────────────────
    auto& actionBarRef() { return actionBar; }
    auto& backpackSlotGuidsRef() { return backpackSlotGuids_; }
    auto& equipSlotGuidsRef() { return equipSlotGuids_; }
    auto& keyringSlotGuidsRef() { return keyringSlotGuids_; }
    auto& containerContentsRef() { return containerContents_; }
    auto& invSlotBaseRef() { return invSlotBase_; }
    auto& packSlotBaseRef() { return packSlotBase_; }
    auto& visibleItemEntryBaseRef() { return visibleItemEntryBase_; }
    auto& visibleItemLayoutVerifiedRef() { return visibleItemLayoutVerified_; }
    auto& visibleItemStrideRef() { return visibleItemStride_; }
    auto& itemInfoCacheRef() { return itemInfoCache_; }
    auto& lastEquipDisplayIdsRef() { return lastEquipDisplayIds_; }
    auto& onlineEquipDirtyRef() { return onlineEquipDirty_; }
    auto& onlineItemsRef() { return onlineItems_; }
    auto& inspectedPlayerItemEntriesRef() { return inspectedPlayerItemEntries_; }
    auto& otherPlayerVisibleDirtyRef() { return otherPlayerVisibleDirty_; }
    auto& otherPlayerVisibleItemEntriesRef() { return otherPlayerVisibleItemEntries_; }
    auto& otherPlayerMoveTimeMsRef() { return otherPlayerMoveTimeMs_; }
    auto& pendingItemPushNotifsRef() { return pendingItemPushNotifs_; }
    auto& pendingItemQueriesRef() { return pendingItemQueries_; }
    auto& pendingMoneyDeltaRef() { return pendingMoneyDelta_; }
    auto& pendingMoneyDeltaTimerRef() { return pendingMoneyDeltaTimer_; }
    auto& pendingAutoInspectRef() { return pendingAutoInspect_; }
    auto& pendingGameObjectLootRetriesRef() { return pendingGameObjectLootRetries_; }
    auto& tempEnchantTimersRef() { return tempEnchantTimers_; }
    auto& localLootStateRef() { return localLootState_; }
    static const auto& getTempEnchantSlotNames() { return kTempEnchantSlotNames; }

    // ── Combat & Player Stats ────────────────────────────────────────
    auto& comboPointsRef() { return comboPoints_; }
    auto& comboTargetRef() { return comboTarget_; }
    auto& isRestingRef() { return isResting_; }
    auto& playerArenaPointsRef() { return playerArenaPoints_; }
    auto& playerArmorRatingRef() { return playerArmorRating_; }
    auto& playerBlockPctRef() { return playerBlockPct_; }
    auto& playerCombatRatingsRef() { return playerCombatRatings_; }
    auto& playerCritPctRef() { return playerCritPct_; }
    auto& playerDodgePctRef() { return playerDodgePct_; }
    auto& playerHealBonusRef() { return playerHealBonus_; }
    auto& playerHonorPointsRef() { return playerHonorPoints_; }
    auto& playerMeleeAPRef() { return playerMeleeAP_; }
    auto& playerMoneyCopperRef() { return playerMoneyCopper_; }
    auto& playerNextLevelXpRef() { return playerNextLevelXp_; }
    auto& playerParryPctRef() { return playerParryPct_; }
    auto& playerRangedAPRef() { return playerRangedAP_; }
    auto& playerRangedCritPctRef() { return playerRangedCritPct_; }
    auto* playerResistancesArr() { return playerResistances_; }
    auto& playerRestedXpRef() { return playerRestedXp_; }
    auto* playerSpellCritPctArr() { return playerSpellCritPct_; }
    auto* playerSpellDmgBonusArr() { return playerSpellDmgBonus_; }
    auto& playerStatsArr() { return playerStats_; }
    auto& playerXpRef() { return playerXp_; }

    // ── Skills ───────────────────────────────────────────────────────
    auto& playerSkillsRef() { return playerSkills_; }
    auto& skillLineAbilityLoadedRef() { return skillLineAbilityLoaded_; }
    auto& skillLineCategoriesRef() { return skillLineCategories_; }
    auto& skillLineDbcLoadedRef() { return skillLineDbcLoaded_; }
    auto& skillLineNamesRef() { return skillLineNames_; }
    auto& spellToSkillLineRef() { return spellToSkillLine_; }

    // ── Spells & Talents ─────────────────────────────────────────────
    auto& activeTalentSpecRef() { return activeTalentSpec_; }
    auto* unspentTalentPointsArr() { return unspentTalentPoints_; }
    auto* learnedTalentsArr() { return learnedTalents_; }
    auto& learnedGlyphsRef() { return learnedGlyphs_; }
    auto& talentsInitializedRef() { return talentsInitialized_; }
    auto& spellFlatModsRef() { return spellFlatMods_; }
    auto& spellPctModsRef() { return spellPctMods_; }
    auto& spellNameCacheRef() { return spellNameCache_; }
    auto& spellNameCacheLoadedRef() { return spellNameCacheLoaded_; }

    // ── Quests & Achievements ────────────────────────────────────────
    auto& completedQuestsRef() { return completedQuests_; }
    auto& npcQuestStatusRef() { return npcQuestStatus_; }
    auto& achievementDatesRef() { return achievementDates_; }
    auto& achievementNameCacheRef() { return achievementNameCache_; }
    auto& earnedAchievementsRef() { return earnedAchievements_; }

    // ── Social, Chat & Contacts ──────────────────────────────────────
    auto& contactsRef() { return contacts_; }
    auto& friendGuidsRef() { return friendGuids_; }
    auto& friendsCacheRef() { return friendsCache; }
    auto& ignoreCacheRef() { return ignoreCache; }
    auto& ignoreListGuidsRef() { return ignoreListGuids_; }
    auto& lastContactListCountRef() { return lastContactListCount_; }
    auto& lastContactListMaskRef() { return lastContactListMask_; }
    auto& lastWhisperSenderRef() { return lastWhisperSender_; }
    auto& lastWhisperSenderGuidRef() { return lastWhisperSenderGuid_; }
    auto& mailInboxRef() { return mailInbox_; }

    // ── World, Map & Zones ───────────────────────────────────────────
    auto& currentMapIdRef() { return currentMapId_; }
    auto& inInstanceRef() { return inInstance_; }
    auto& worldStateMapIdRef() { return worldStateMapId_; }
    auto& worldStatesRef() { return worldStates_; }
    auto& worldStateZoneIdRef() { return worldStateZoneId_; }
    auto& minimapPingsRef() { return minimapPings_; }
    auto& gossipPoisRef() { return gossipPois_; }
    auto& playerExploredZonesRef() { return playerExploredZones_; }
    auto& hasPlayerExploredZonesRef() { return hasPlayerExploredZones_; }
    auto& factionStandingsRef() { return factionStandings_; }
    auto& initialFactionsRef() { return initialFactions_; }
    auto& watchedFactionIdRef() { return watchedFactionId_; }

    // ── Corpse & Home Bind ───────────────────────────────────────────
    auto& corpseGuidRef() { return corpseGuid_; }
    auto& corpseMapIdRef() { return corpseMapId_; }
    auto& corpsePositionValidRef() { return corpsePositionValid_; }
    auto& corpseReclaimAvailableMsRef() { return corpseReclaimAvailableMs_; }
    auto& corpseXRef() { return corpseX_; }
    auto& corpseYRef() { return corpseY_; }
    auto& corpseZRef() { return corpseZ_; }
    auto& hasHomeBindRef() { return hasHomeBind_; }
    auto& homeBindMapIdRef() { return homeBindMapId_; }
    auto& homeBindPosRef() { return homeBindPos_; }

    // ── Area Triggers ────────────────────────────────────────────────
    auto& activeAreaTriggersRef() { return activeAreaTriggers_; }
    auto& areaTriggerCheckTimerRef() { return areaTriggerCheckTimer_; }
    auto& areaTriggerDbcLoadedRef() { return areaTriggerDbcLoaded_; }
    auto& areaTriggerMsgsRef() { return areaTriggerMsgs_; }
    auto& areaTriggersRef() { return areaTriggers_; }
    auto& areaTriggerSuppressFirstRef() { return areaTriggerSuppressFirst_; }
    auto& areaTriggerCooldownRef() { return areaTriggerCooldown_; }

    // ── Death & Resurrection ─────────────────────────────────────────
    auto& playerDeadRef() { return playerDead_; }
    auto& releasedSpiritRef() { return releasedSpirit_; }
    auto& repopPendingRef() { return repopPending_; }
    auto& lastRepopRequestMsRef() { return lastRepopRequestMs_; }
    auto& pendingSpiritHealerGuidRef() { return pendingSpiritHealerGuid_; }
    auto& resurrectCasterGuidRef() { return resurrectCasterGuid_; }
    auto& resurrectIsSpiritHealerRef() { return resurrectIsSpiritHealer_; }
    auto& resurrectPendingRef() { return resurrectPending_; }
    auto& resurrectRequestPendingRef() { return resurrectRequestPending_; }
    auto& selfResAvailableRef() { return selfResAvailable_; }

    // ── Summon & Battlefield ─────────────────────────────────────────
    auto& pendingSummonRequestRef() { return pendingSummonRequest_; }
    auto& summonerGuidRef() { return summonerGuid_; }
    auto& summonerNameRef() { return summonerName_; }
    auto& summonTimeoutSecRef() { return summonTimeoutSec_; }
    auto& bfMgrInvitePendingRef() { return bfMgrInvitePending_; }

    // ── Pet & Stable ─────────────────────────────────────────────────
    auto& petActionSlotsRef() { return petActionSlots_; }
    auto& petAutocastSpellsRef() { return petAutocastSpells_; }
    auto& petCommandRef() { return petCommand_; }
    auto& petGuidRef() { return petGuid_; }
    auto& petReactRef() { return petReact_; }
    auto& petSpellListRef() { return petSpellList_; }
    auto& stabledPetsRef() { return stabledPets_; }
    auto& stableMasterGuidRef() { return stableMasterGuid_; }
    auto& stableNumSlotsRef() { return stableNumSlots_; }
    auto& stableWindowOpenRef() { return stableWindowOpen_; }

    // ── Trainer, GM & Misc ───────────────────────────────────────────
    auto& currentTrainerListRef() { return currentTrainerList_; }
    auto& trainerTabsRef() { return trainerTabs_; }
    auto& gmTicketActiveRef() { return gmTicketActive_; }
    auto& gmTicketTextRef() { return gmTicketText_; }
    auto& bookPagesRef() { return bookPages_; }
    auto& activeTotemSlotsRef() { return activeTotemSlots_; }
    auto& unitAurasCacheRef() { return unitAurasCache_; }
    auto& lastInteractedGoGuidRef() { return lastInteractedGoGuid_; }
    auto& pendingGameObjectInteractGuidRef() { return pendingGameObjectInteractGuid_; }

    // ── Tab Cycling ──────────────────────────────────────────────────
    auto& tabCycleIndexRef() { return tabCycleIndex; }
    auto& tabCycleListRef() { return tabCycleList; }
    auto& tabCycleStaleRef() { return tabCycleStale; }

    // ── UI & Event Callbacks ─────────────────────────────────────────
    auto& achievementEarnedCallbackRef() { return achievementEarnedCallback_; }
    auto& addonChatCallbackRef() { return addonChatCallback_; }
    auto& addonEventCallbackRef() { return addonEventCallback_; }
    auto& appearanceChangedCallbackRef() { return appearanceChangedCallback_; }
    auto& playerModelRebuildCallbackRef() { return playerModelRebuildCallback_; }
    auto& autoFollowCallbackRef() { return autoFollowCallback_; }
    auto& chargeCallbackRef() { return chargeCallback_; }
    auto& chatBubbleCallbackRef() { return chatBubbleCallback_; }
    auto& creatureDespawnCallbackRef() { return creatureDespawnCallback_; }
    auto& creatureMoveCallbackRef() { return creatureMoveCallback_; }
    auto& creatureSpawnCallbackRef() { return creatureSpawnCallback_; }
    auto& emoteAnimCallbackRef() { return emoteAnimCallback_; }
    auto& gameObjectDespawnCallbackRef() { return gameObjectDespawnCallback_; }
    auto& gameObjectInfoCallbackRef() { return gameObjectInfoCallback_; }
    auto& gameObjectMoveCallbackRef() { return gameObjectMoveCallback_; }
    auto& gameObjectSpawnCallbackRef() { return gameObjectSpawnCallback_; }
    auto& gameObjectStateCallbackRef() { return gameObjectStateCallback_; }
    auto& ghostStateCallbackRef() { return ghostStateCallback_; }
    auto& hearthstonePreloadCallbackRef() { return hearthstonePreloadCallback_; }
    auto& hitReactionCallbackRef() { return hitReactionCallback_; }
    auto& itemLootCallbackRef() { return itemLootCallback_; }
    auto& knockBackCallbackRef() { return knockBackCallback_; }
    auto& lootWindowCallbackRef() { return lootWindowCallback_; }
    auto& meleeSwingCallbackRef() { return meleeSwingCallback_; }
    auto& rangedWeaponSwapCallbackRef() { return rangedWeaponSwapCallback_; }
    void suppressNextMeleeSwingAnim() { suppressMeleeSwingAnim_ = true; }
    bool consumeSuppressMeleeSwingAnim() {
        bool v = suppressMeleeSwingAnim_;
        suppressMeleeSwingAnim_ = false;
        return v;
    }
    auto& mountCallbackRef() { return mountCallback_; }
    auto& npcAggroCallbackRef() { return npcAggroCallback_; }
    auto& npcDeathCallbackRef() { return npcDeathCallback_; }
    auto& npcFarewellCallbackRef() { return npcFarewellCallback_; }
    auto& npcGreetingCallbackRef() { return npcGreetingCallback_; }
    auto& npcRespawnCallbackRef() { return npcRespawnCallback_; }
    auto& npcSwingCallbackRef() { return npcSwingCallback_; }
    auto& npcVendorCallbackRef() { return npcVendorCallback_; }
    auto& openLfgCallbackRef() { return openLfgCallback_; }
    auto& otherPlayerLevelUpCallbackRef() { return otherPlayerLevelUpCallback_; }
    auto& otherPlayerMountCallbackRef() { return otherPlayerMountCallback_; }
    auto& playerDespawnCallbackRef() { return playerDespawnCallback_; }
    auto& playerEquipmentCallbackRef() { return playerEquipmentCallback_; }
    auto& playerHealthCallbackRef() { return playerHealthCallback_; }
    auto& playerSpawnCallbackRef() { return playerSpawnCallback_; }
    auto& pvpHonorCallbackRef() { return pvpHonorCallback_; }
    auto& questCompleteCallbackRef() { return questCompleteCallback_; }
    auto& questProgressCallbackRef() { return questProgressCallback_; }
    auto& repChangeCallbackRef() { return repChangeCallback_; }
    auto& spellCastAnimCallbackRef() { return spellCastAnimCallback_; }
    auto& spellCastFailedCallbackRef() { return spellCastFailedCallback_; }
    auto& sprintAuraCallbackRef() { return sprintAuraCallback_; }
    auto& stealthStateCallbackRef() { return stealthStateCallback_; }
    auto& stunStateCallbackRef() { return stunStateCallback_; }
    auto& taxiFlightStartCallbackRef() { return taxiFlightStartCallback_; }
    auto& taxiOrientationCallbackRef() { return taxiOrientationCallback_; }
    auto& taxiLandingPositionCallbackRef() { return taxiLandingPositionCallback_; }
    auto& taxiPrecacheCallbackRef() { return taxiPrecacheCallback_; }
    auto& transportMoveCallbackRef() { return transportMoveCallback_; }
    auto& unitAnimHintCallbackRef() { return unitAnimHintCallback_; }
    auto& unitMoveFlagsCallbackRef() { return unitMoveFlagsCallback_; }
    auto& worldEntryCallbackRef() { return worldEntryCallback_; }

    // ── Methods moved from private (domain handler use) ──────────────
    void addCombatText(CombatTextEntry::Type type, int32_t amount, uint32_t spellId,
                       bool isPlayerSource, uint8_t powerType = 0,
                       uint64_t srcGuid = 0, uint64_t dstGuid = 0);
    bool shouldLogSpellstealAura(uint64_t casterGuid, uint64_t victimGuid, uint32_t spellId);
    void addSystemChatMessage(const std::string& message);
    void sendPing();
    void setTransportAttachment(uint64_t childGuid, ObjectType type,
                                uint64_t transportGuid, const glm::vec3& localOffset,
                                bool hasLocalOrientation, float localOrientation,
                                bool offsetNeedsTransportResolution = false);
    void clearTransportAttachment(uint64_t childGuid);
    std::string guidToUnitId(uint64_t guid) const;
    Unit* getUnitByGuid(uint64_t guid);
    uint64_t resolveOnlineItemGuid(uint32_t itemId) const;
    void rebuildOnlineInventory();
    void maybeDetectVisibleItemLayout();
    void updateOtherPlayerVisibleItems(uint64_t guid, const FlatFieldMap& fields);
    void cacheInspectedPlayerEquipment(uint64_t guid, const std::array<uint32_t, 19>& itemEntries);
    void detectInventorySlotBases(const FlatFieldMap& fields);
    bool applyInventoryFields(const FlatFieldMap& fields);
    void extractContainerFields(uint64_t containerGuid, const FlatFieldMap& fields);
    void extractSkillFields(const FlatFieldMap& fields);
    void extractExploredZoneFields(const FlatFieldMap& fields);
    void applyQuestStateFromFields(const FlatFieldMap& fields);
    void sanitizeMovementForTaxi();
    void loadSpellNameCache() const;
    void loadFactionNameCache() const;
    void loadAchievementNameCache();
    void loadSkillLineDbc();
    void loadSkillLineAbilityDbc();
    std::string getFactionName(uint32_t factionId) const;
    std::string getLfgDungeonName(uint32_t dungeonId) const;
    void queryItemInfo(uint32_t entry, uint64_t guid);

    // --- Inner types exposed for former friend classes ---
    struct TransportAttachment {
        ObjectType type = ObjectType::OBJECT;
        uint64_t transportGuid = 0;
        glm::vec3 localOffset{0.0f};
        float localOrientation = 0.0f;
        bool hasLocalOrientation = false;
        bool offsetNeedsTransportResolution = false;
    };
    struct AreaTriggerEntry {
        uint32_t id = 0;
        uint32_t mapId = 0;
        float x = 0, y = 0, z = 0;
        float radius = 0;
        float boxLength = 0, boxWidth = 0, boxHeight = 0;
        float boxYaw = 0;
    };
    struct PendingLootRetry {
        uint64_t guid = 0;
        float timer = 0.0f;
        uint8_t remainingRetries = 0;
        bool sendLoot = false;
    };
    struct SpellReagent { uint32_t itemId = 0; uint32_t count = 0; };
    struct SpellNameEntry {
        std::string name; std::string rank; std::string description;
        uint32_t schoolMask = 0; uint8_t dispelType = 0; uint32_t attrEx = 0;
        // Spell.dbc Targets bitmask (SpellCastTargetFlags) — 0x10 = TARGET_FLAG_ITEM
        uint32_t targetFlags = 0;
        // Spell.dbc RangeIndex resolved against SpellRange.dbc. A max range of 0
        // means "Self Only" (shouts, self-buffs); negative means SpellRange.dbc
        // was unavailable, so callers should not infer anything from it.
        float maxRange = -1.0f;
        int32_t effectBasePoints[3] = {0, 0, 0};
        uint32_t effectIds[3] = {0, 0, 0};
        // Spell.dbc EffectImplicitTargetA — what the spell expects to be aimed
        // at. 21 means a friendly unit, which is how heals and buffs are told
        // apart from damage that shares the same effect and school.
        uint32_t implicitTargetA = 0;
        float durationSec = 0.0f;
        uint32_t spellVisualId = 0;
        uint32_t recoveryMs = 0;
        uint32_t categoryRecoveryMs = 0;
        uint32_t createdItemId = 0;
        SpellReagent reagents[8] = {};
        uint32_t trivialSkillHigh = 0;
        uint32_t trivialSkillLow = 0;
        uint32_t minSkillRank = 0;
        // Spell.dbc TargetAuraState: the aura state the target must be in for the spell
        // to be castable (e.g. Execute → AURA_STATE_HEALTHLESS_20_PERCENT). 0 = none.
        uint32_t targetAuraState = 0;
    };
    static constexpr size_t PLAYER_EXPLORED_ZONES_COUNT = 128;
    std::string getAreaName(uint32_t areaId) const;
    struct OnlineItemInfo {
        uint32_t entry = 0;
        uint32_t stackCount = 1;
        uint32_t curDurability = 0;
        uint32_t maxDurability = 0;
        uint32_t permanentEnchantId = 0;
        uint32_t temporaryEnchantId = 0;
        std::array<uint32_t, 3> socketEnchantIds{};
        uint32_t flags = 0;              // ITEM_FIELD_FLAGS (bit 0x1 = soulbound)
        int32_t  randomPropertyId = 0;   // ITEM_FIELD_RANDOM_PROPERTIES_ID (signed)
        uint32_t suffixFactor = 0;       // ITEM_FIELD_PROPERTY_SEED (random-suffix stat scale)
    };
    bool isHostileFaction(uint32_t factionTemplateId) const {
        auto it = factionHostileMap_.find(factionTemplateId);
        return it != factionHostileMap_.end() ? it->second : true;
    }

private:
    // Dead: autoTargetAttacker moved to CombatHandler

    /**
     * Handle incoming packet from world server
     */
    void handlePacket(network::Packet& packet);
    void registerOpcodeHandlers();
    void registerSkipHandler(LogicalOpcode op);
    void registerErrorHandler(LogicalOpcode op, const char* msg);
    void registerHandler(LogicalOpcode op, void (GameHandler::*handler)(network::Packet&));
    void registerWorldHandler(LogicalOpcode op, void (GameHandler::*handler)(network::Packet&));
    void enqueueIncomingPacket(const network::Packet& packet);
    void enqueueIncomingPacketFront(network::Packet&& packet);
    void processQueuedIncomingPackets();

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
     * Handle SMSG_CHARACTER_LOGIN_FAILED from server
     */
    void handleCharLoginFailed(network::Packet& packet);

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

    /**
     * Handle SMSG_ACCOUNT_DATA_TIMES from server
     */
    void handleAccountDataTimes(network::Packet& packet);

    /**
     * Handle SMSG_MOTD from server
     */
    void handleMotd(network::Packet& packet);

    /** Handle SMSG_NOTIFICATION (vanilla/classic server notification string) */
    void handleNotification(network::Packet& packet);

    /**
     * Handle SMSG_PONG from server
     */
    void handlePong(network::Packet& packet);

    void handleItemQueryResponse(network::Packet& packet);
    void emitOtherPlayerEquipment(uint64_t guid);
    void emitAllOtherPlayerEquipment();

    // handleAttackStart, handleAttackStop, handleAttackerStateUpdate,
    // handleSpellDamageLog, handleSpellHealLog removed

    // ---- Equipment set handler ----
    void handleUpdateAuraDuration(uint8_t slot, uint32_t durationMs);
    // handleSetForcedReactions — dispatched via CombatHandler

    // ---- Guild handlers ----
    void handlePetSpells(network::Packet& packet);

    // ---- Character creation handler ----
    void handleCharCreateResponse(network::Packet& packet);

    // ---- XP handler ----
    void handleXpGain(network::Packet& packet);

    // ---- Creature movement handler ----

    // ---- Other player movement (MSG_MOVE_* from server) ----

    void clearPendingQuestAccept(uint32_t questId);
    void triggerQuestAcceptResync(uint32_t questId, uint64_t npcGuid, const char* reason);
    bool hasQuestInLog(uint32_t questId) const;
    std::string getQuestTitle(uint32_t questId) const;
    const QuestLogEntry* findQuestLogEntry(uint32_t questId) const;
    int findQuestLogSlotIndexFromServer(uint32_t questId) const;
    void addQuestToLocalLogIfMissing(uint32_t questId, const std::string& title, const std::string& objectives);
    bool resyncQuestLogFromServerSlots(bool forceQueryMetadata);
    void addMoneyCopper(uint32_t amount);

    // ---- Teleport handler ----

    // ---- Movement ACK handlers ----

    // ---- Area trigger detection ----
    void loadAreaTriggerDbc();
    void checkAreaTriggers();

    // ---- Instance lockout handler ----
    void handleSummonRequest(network::Packet& packet);
    void resetTradeState();
    void handleDuelRequested(network::Packet& packet);
    void handleDuelComplete(network::Packet& packet);
    void handleDuelWinner(network::Packet& packet);

    // ---- LFG / Dungeon Finder handlers ----

    // ---- Arena / Battleground handlers ----

    // ---- Bank handlers ----

    // ---- Guild Bank handlers ----

    // ---- Auction House handlers ----

    // ---- Mail handlers ----

    // ---- Taxi handlers ----

    // ---- Server info handlers ----
    void handleQueryTimeResponse(network::Packet& packet);

    // ---- Social handlers ----

    // ---- Logout handlers ----



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

    // Explicit service dependencies (owned by Application)
    GameServices& services_;

    // Domain handlers — each manages a specific concern extracted from GameHandler
    std::unique_ptr<ChatHandler>      chatHandler_;
    std::unique_ptr<MovementHandler>  movementHandler_;
    std::unique_ptr<CombatHandler>    combatHandler_;
    std::unique_ptr<SpellHandler>     spellHandler_;
    std::unique_ptr<InventoryHandler> inventoryHandler_;
    std::unique_ptr<SocialHandler>    socialHandler_;
    std::unique_ptr<QuestHandler>     questHandler_;
    std::unique_ptr<WardenHandler>    wardenHandler_;

    // Opcode dispatch table — built once in registerOpcodeHandlers(), called by handlePacket()
    using PacketHandler = std::function<void(network::Packet&)>;
    std::unordered_map<LogicalOpcode, PacketHandler> dispatchTable_;

    // Opcode translation table (expansion-specific wire ↔ logical mapping)
    OpcodeTable opcodeTable_;

    // Update field table (expansion-specific field index mapping)
    UpdateFieldTable updateFieldTable_;

    // Packet parsers (expansion-specific binary format handling)
    std::unique_ptr<PacketParsers> packetParsers_;

    // Network
    std::unique_ptr<network::WorldSocket> socket;
    std::deque<network::Packet> pendingIncomingPackets_;

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
    std::chrono::steady_clock::time_point movementClockStart_ = std::chrono::steady_clock::now();
    uint32_t lastMovementTimestampMs_ = 0;
    bool serverMovementAllowed_ = true;
    uint32_t monsterMovePacketsThisTick_ = 0;
    uint32_t monsterMovePacketsDroppedThisTick_ = 0;

    // Fall/jump tracking for movement packet correctness.
    // fallTime must be the elapsed ms since the FALLING flag was set; the server
    // uses it for fall-damage calculations and anti-cheat validation.
    bool isFalling_ = false;
    uint32_t fallStartMs_ = 0;  // movementInfo.time value when FALLING started

    // Inventory
    Inventory inventory;

    // Entity tracking (delegated to EntityController)
    std::unique_ptr<EntityController> entityController_;

    // Chat (state lives in ChatHandler; callbacks remain here for cross-domain access)
    ChatBubbleCallback chatBubbleCallback_;
    AddonChatCallback addonChatCallback_;
    AddonEventCallback addonEventCallback_;
    SpellIconPathResolver spellIconPathResolver_;
    ItemIconPathResolver itemIconPathResolver_;
    SpellDataResolver spellDataResolver_;
    RandomPropertyNameResolver randomPropertyNameResolver_;
    RandomStatResolver randomStatResolver_;
    EmoteAnimCallback emoteAnimCallback_;

    // Targeting
    uint64_t targetGuid = 0;
    uint64_t focusGuid = 0;              // Focus target
    uint64_t lastTargetGuid = 0;         // Previous target
    uint64_t mouseoverGuid_ = 0;         // Set each frame by nameplate renderer
    std::vector<uint64_t> tabCycleList;
    int tabCycleIndex = -1;
    bool tabCycleStale = true;

    // Heartbeat
    uint32_t pingSequence = 0;               // Ping sequence number (increments)
    float timeSinceLastPing = 0.0f;          // Time since last ping sent (seconds)
    float pingInterval = 30.0f;              // Ping interval (30 seconds)
    float timeSinceLastMoveHeartbeat_ = 0.0f; // Periodic movement heartbeat to keep server position synced
    float moveHeartbeatInterval_ = 0.5f;
    uint32_t lastHeartbeatSendTimeMs_ = 0;
    float lastHeartbeatX_ = 0.0f;
    float lastHeartbeatY_ = 0.0f;
    float lastHeartbeatZ_ = 0.0f;
    uint32_t lastHeartbeatFlags_ = 0;
    uint64_t lastHeartbeatTransportGuid_ = 0;
    uint32_t lastNonHeartbeatMoveSendTimeMs_ = 0;
    uint32_t lastFacingSendTimeMs_ = 0;
    float lastFacingSentOrientation_ = 0.0f;
    uint32_t lastLatency = 0;                // Last measured latency (milliseconds)
    std::chrono::steady_clock::time_point pingTimestamp_;  // Time CMSG_PING was sent

    // Player GUID and map
    uint64_t playerGuid = 0;
    uint32_t currentMapId_ = 0;
    bool hasHomeBind_ = false;
    uint32_t homeBindMapId_ = 0;
    uint32_t homeBindZoneId_ = 0;
    glm::vec3 homeBindPos_{0.0f};


    // ---- Friend/contact list cache ----
    std::unordered_map<std::string, uint64_t> friendsCache;  // name -> guid
    std::unordered_set<uint64_t> friendGuids_;               // all known friend GUIDs (for name backfill)
    uint32_t lastContactListMask_ = 0;
    uint32_t lastContactListCount_ = 0;
    std::vector<ContactEntry> contacts_;                     // structured contact list (friends + ignores)

    // ---- World state and faction initialization snapshots ----
    uint32_t worldStateMapId_ = 0;
    uint32_t worldStateZoneId_ = 0;
    // The active non-combat companion and the spell that called it. Re-casting
    // that spell dismisses rather than re-summons, which is how a companion is
    // put away: it has no aura to cancel from the buff bar, so this toggle is
    // the only way to send it back.
    uint64_t activeCritterGuid_ = 0;
    uint32_t activeCritterSpellId_ = 0;
    std::unordered_map<uint32_t, uint32_t> worldStates_;
    std::vector<FactionStandingInit> initialFactions_;

    // ---- Ignore list cache ----
    std::unordered_map<std::string, uint64_t> ignoreCache;  // name -> guid (UI display)
    std::unordered_set<uint64_t> ignoreListGuids_;            // authoritative GUID set from server

    // ---- Logout state ----
    bool  loggingOut_        = false;
    float logoutCountdown_   = 0.0f;   // seconds remaining before server logs us out (0 = instant/done)

    // ---- Display state ----
    bool helmVisible_ = true;
    bool cloakVisible_ = true;
    uint8_t standState_ = 0;  // 0=stand, 1=sit, ..., 7=dead, 8=kneel (server-confirmed)

    // ---- Follow state ----
    uint64_t followTargetGuid_ = 0;
    glm::vec3 followRenderPos_{0.0f};  // Render-space position of followed entity (updated each frame)

    // ---- AFK/DND status ----
    bool afkStatus_ = false;
    bool dndStatus_ = false;
    std::string afkMessage_;
    std::string dndMessage_;
    std::string lastWhisperSender_;
    uint64_t lastWhisperSenderGuid_ = 0;

    // ---- Online item tracking ----
    std::unordered_map<uint64_t, OnlineItemInfo> onlineItems_;
    std::unordered_map<uint32_t, ItemQueryResponseData> itemInfoCache_;
    std::unordered_set<uint32_t> pendingItemQueries_;
    float pendingItemQueryTimer_ = 0.0f;

    // Deferred SMSG_ITEM_PUSH_RESULT notifications for items whose info wasn't
    // cached at arrival time; emitted once the query response arrives.
    struct PendingItemPushNotif {
        uint32_t itemId = 0;
        uint32_t count  = 1;
    };
    std::vector<PendingItemPushNotif> pendingItemPushNotifs_;
    std::array<uint64_t, 23> equipSlotGuids_{};
    std::array<uint64_t, 16> backpackSlotGuids_{};
    std::array<uint64_t, 32> keyringSlotGuids_{};
    // Container (bag) contents: containerGuid -> array of item GUIDs per slot
    struct ContainerInfo {
        uint32_t numSlots = 0;
        std::array<uint64_t, 36> slotGuids{};  // max 36 slots
    };
    std::unordered_map<uint64_t, ContainerInfo> containerContents_;
    int invSlotBase_ = -1;
    int packSlotBase_ = -1;
    FlatFieldMap lastPlayerFields_;
    bool onlineEquipDirty_ = false;
    std::array<uint32_t, 19> lastEquipDisplayIds_{};

    // Visible equipment for other players: detect the update-field layout (base + stride)
    // using the local player's own equipped items, then decode other players by index.
    // WotLK 3.3.5a (AzerothCore/ChromieCraft): visible item entries appear at field
    // WotLK 3.3.5a: PLAYER_VISIBLE_ITEM_1_ENTRYID = field 283, stride 2.
    // Confirmed by RAW FIELDS dump: base=283 gives 17/19 valid item IDs,
    // base=284 reads enchant values instead.
    // Slots: HEAD=0, NECK=1, SHOULDERS=2, BODY=3, CHEST=4, WAIST=5, LEGS=6,
    //        FEET=7, WRISTS=8, HANDS=9, FINGER1=10, FINGER2=11, TRINKET1=12,
    //        TRINKET2=13, BACK=14, MAINHAND=15, OFFHAND=16, RANGED=17, TABARD=18
    int visibleItemEntryBase_ = 283;
    int visibleItemStride_ = 2;
    bool visibleItemLayoutVerified_ = false;  // true once heuristic confirms/overrides default
    std::unordered_map<uint64_t, std::array<uint32_t, 19>> otherPlayerVisibleItemEntries_;
    std::unordered_set<uint64_t> otherPlayerVisibleDirty_;
    std::unordered_map<uint64_t, uint32_t> otherPlayerMoveTimeMs_;
    std::unordered_map<uint64_t, float>    otherPlayerSmoothedIntervalMs_;  // EMA of packet intervals

    // Inspect fallback (when visible item fields are missing/unreliable)
    std::unordered_map<uint64_t, std::array<uint32_t, 19>> inspectedPlayerItemEntries_;
    InspectResult inspectResult_; // most-recently received inspect response
    std::unordered_set<uint64_t> pendingAutoInspect_;
    float inspectRateLimit_ = 0.0f;

    // ---- Combat ----
    bool wasCombat_ = false;  // Previous frame combat state for PLAYER_REGEN edge detection
    std::deque<std::string>    areaTriggerMsgs_;

    WorldEntryCallback worldEntryCallback_;
    KnockBackCallback knockBackCallback_;
    CameraShakeCallback cameraShakeCallback_;
    AutoFollowCallback autoFollowCallback_;
    UnstuckCallback unstuckCallback_;
    UnstuckCallback unstuckGyCallback_;
    UnstuckCallback unstuckHearthCallback_;
    BindPointCallback bindPointCallback_;
    HearthstonePreloadCallback hearthstonePreloadCallback_;
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
    GameObjectInfoCallback gameObjectInfoCallback_;
    GameObjectCustomAnimCallback gameObjectCustomAnimCallback_;
    GameObjectStateCallback gameObjectStateCallback_;
    SprintAuraCallback sprintAuraCallback_;
    VehicleStateCallback vehicleStateCallback_;

    // Transport tracking
    std::unordered_map<uint64_t, TransportAttachment> transportAttachments_;
    // Transport GUID tracking moved to EntityController
    uint64_t playerTransportGuid_ = 0;             // Transport the player is riding (0 = none)
    glm::vec3 playerTransportOffset_ = glm::vec3(0.0f); // Player offset on transport
    uint64_t playerTransportStickyGuid_ = 0;       // Last transport player was on (temporary retention)
    float playerTransportStickyTimer_ = 0.0f;      // Seconds to keep sticky transport alive after transient clears
    // Consecutive frames a ship rider has had no deck beneath them. Walking off
    // onto the pier leaves the rider well inside the ship's boarding footprint,
    // so losing the deck is what actually distinguishes "ashore" from "aboard".
    // Counted rather than tested instantly so a jump is not a disembark.
    int shipNoDeckSupportFrames_ = 0;
    bool pendingPlayerTransportTransfer_ = false;
    uint64_t pendingPlayerTransportGuid_ = 0;
    uint32_t pendingPlayerTransportEntry_ = 0;
    uint32_t pendingPlayerTransportMapId_ = 0xFFFFFFFFu;
    glm::vec3 pendingPlayerTransportOffset_ = glm::vec3(0.0f);
    std::unique_ptr<TransportManager> transportManager_;  // Transport movement manager
    uint32_t weaponProficiency_ = 0;  // bitmask from SMSG_SET_PROFICIENCY itemClass=2
    uint32_t armorProficiency_  = 0;  // bitmask from SMSG_SET_PROFICIENCY itemClass=4
    std::vector<MinimapPing> minimapPings_;
    uint64_t pendingGameObjectInteractGuid_ = 0;
    // Owned fishing bobber whose bite animation has fired. Keeping this explicit
    // lets the UI target/reel it even while GAMEOBJECT_QUERY metadata is pending.
    uint64_t hookedFishingBobberGuid_ = 0;

    // Talents (dual-spec support)
    uint8_t activeTalentSpec_ = 0;                              // Currently active spec (0 or 1)
    uint8_t unspentTalentPoints_[2] = {0, 0};                   // Unspent points per spec
    std::unordered_map<uint32_t, uint8_t> learnedTalents_[2];  // Learned talents per spec
    std::array<std::array<uint16_t, MAX_GLYPH_SLOTS>, 2> learnedGlyphs_{};  // Glyphs per spec
    std::unordered_map<uint32_t, TalentEntry> talentCache_;      // talentId -> entry
    std::unordered_map<uint32_t, TalentTabEntry> talentTabCache_; // tabId -> entry
    bool talentDbcLoaded_ = false;
    bool talentsInitialized_ = false;                           // Reset on world entry; guards first-spec selection

    // ---- Area trigger detection ----
    bool areaTriggerDbcLoaded_ = false;
    std::vector<AreaTriggerEntry> areaTriggers_;
    std::unordered_set<uint32_t> activeAreaTriggers_;  // triggers player is currently inside
    float areaTriggerCheckTimer_ = 0.0f;
    bool areaTriggerSuppressFirst_ = false;  // suppress first check after map transfer
    float areaTriggerCooldown_ = 0.0f;       // seconds remaining — suppress ALL triggers

    // Craft queue: seconds the expired cast bar has waited for SMSG_SPELL_GO
    // (which re-casts the next queued item) before giving up
    float craftCastGoGraceSec_ = 0.0f;

    std::array<ActionBarSlot, ACTION_BAR_SLOTS> actionBar{};
    std::unordered_map<uint32_t, std::string> macros_;  // client-side macro text (persisted in char config)
    std::vector<AuraSlot> playerAuras;
    std::vector<AuraSlot> targetAuras;
    std::unordered_map<uint64_t, std::vector<AuraSlot>> unitAurasCache_; // per-unit aura cache
    uint64_t petGuid_ = 0;
    uint32_t petActionSlots_[10] = {};   // SMSG_PET_SPELLS action bar (10 slots)
    uint8_t  petCommand_ = 1;            // 0=stay,1=follow,2=attack,3=dismiss
    uint8_t  petReact_   = 1;            // 0=passive,1=defensive,2=aggressive
    bool     petRenameablePending_ = false;  // set by SMSG_PET_RENAMEABLE, consumed by UI
    std::vector<uint32_t> petSpellList_; // known pet spells
    std::unordered_set<uint32_t> petAutocastSpells_;  // spells with autocast on

    // ---- Pet Stable ----
    bool stableWindowOpen_    = false;
    uint64_t stableMasterGuid_ = 0;
    uint8_t  stableNumSlots_   = 0;
    std::vector<StabledPet> stabledPets_;
    void handleListStabledPets(network::Packet& packet);

    // ---- Battleground queue state ----
    std::array<BgQueueSlot, 3> bgQueues_{};

    // ---- Available battleground list (SMSG_BATTLEFIELD_LIST) ----
    std::vector<AvailableBgInfo> availableBgs_;

    // Instance difficulty
    uint32_t instanceDifficulty_ = 0;
    bool instanceIsHeroic_ = false;
    bool inInstance_ = false;

    // Mirror timers (0=fatigue, 1=breath, 2=feigndeath)
    MirrorTimer mirrorTimers_[3];

    // Shapeshift form (from UNIT_FIELD_BYTES_1 byte 3)
    uint8_t  shapeshiftFormId_ = 0;
    // Combo points (rogues/druids)
    uint8_t  comboPoints_ = 0;
    uint64_t comboTarget_ = 0;

    // Instance / raid lockouts
    std::vector<InstanceLockout> instanceLockouts_;

    // Arena team stats (indexed by team slot, updated by SMSG_ARENA_TEAM_STATS)
    std::vector<ArenaTeamStats>  arenaTeamStats_;
    // Arena team rosters (updated by SMSG_ARENA_TEAM_ROSTER)
    std::vector<ArenaTeamRoster> arenaTeamRosters_;

    // BG scoreboard (MSG_PVP_LOG_DATA)
    BgScoreboardData bgScoreboard_;

    // BG flag carrier / player positions (MSG_BATTLEGROUND_PLAYER_POSITIONS)
    std::vector<BgPlayerPosition> bgPlayerPositions_;

    // Instance encounter boss units (slots 0-4 from SMSG_UPDATE_INSTANCE_ENCOUNTER_UNIT)
    std::array<uint64_t, kMaxEncounterSlots> encounterUnitGuids_ = {};  // 0 = empty slot

    // LFG / Dungeon Finder state
    LfgState lfgState_        = LfgState::None;
    uint32_t lfgDungeonId_    = 0;   // current dungeon entry
    uint32_t lfgProposalId_   = 0;   // pending proposal id (0 = none)
    int32_t  lfgAvgWaitSec_   = -1;  // estimated wait, -1=unknown
    uint32_t lfgTimeInQueueMs_= 0;   // ms already in queue
    uint32_t lfgBootVotes_    = 0;   // current boot-yes votes
    uint32_t lfgBootTotal_    = 0;   // total votes cast
    uint32_t lfgBootTimeLeft_ = 0;   // seconds remaining
    uint32_t lfgBootNeeded_   = 0;   // votes needed to kick
    std::string lfgBootTargetName_;  // name of player being voted on
    std::string lfgBootReason_;      // reason given for kick

    // Ready check state
    bool        pendingReadyCheck_       = false;
    uint32_t    readyCheckReadyCount_    = 0;
    uint32_t    readyCheckNotReadyCount_ = 0;
    std::string readyCheckInitiator_;
    std::vector<ReadyCheckResult> readyCheckResults_; // per-player status live during check

    // Faction standings (factionId → absolute standing value)
    std::unordered_map<uint32_t, int32_t> factionStandings_;
    // Faction name cache (factionId → name), populated lazily from Faction.dbc
    mutable std::unordered_map<uint32_t, std::string> factionNameCache_;
    // repListId → factionId mapping (populated with factionNameCache)
    mutable std::unordered_map<uint32_t, uint32_t> factionRepListToId_;
    // factionId → repListId reverse mapping
    mutable std::unordered_map<uint32_t, uint32_t> factionIdToRepList_;
    mutable bool factionNameCacheLoaded_ = false;

    // ---- Group ----
    GroupListData partyData;
    bool pendingGroupInvite = false;
    std::string pendingInviterName;

    // Item text state
    bool        itemTextOpen_   = false;
    std::string itemText_;

    // Shared quest state
    bool        pendingSharedQuest_       = false;
    uint32_t    sharedQuestId_            = 0;
    std::string sharedQuestTitle_;
    std::string sharedQuestSharerName_;
    uint64_t    sharedQuestSharerGuid_    = 0;

    // Summon state
    bool        pendingSummonRequest_ = false;
    uint64_t    summonerGuid_         = 0;
    std::string summonerName_;
    float       summonTimeoutSec_     = 0.0f;
    uint32_t    totalTimePlayed_      = 0;
    uint32_t    levelTimePlayed_      = 0;

    // Who results (last SMSG_WHO response)
    std::vector<WhoEntry> whoResults_;
    uint32_t whoOnlineCount_ = 0;

    // Trade state
    TradeStatus tradeStatus_  = TradeStatus::None;
    uint64_t    tradePeerGuid_= 0;
    std::string tradePeerName_;
    std::array<TradeSlot, TRADE_SLOT_COUNT> myTradeSlots_{};
    std::array<TradeSlot, TRADE_SLOT_COUNT> peerTradeSlots_{};
    uint64_t myTradeGold_   = 0;
    uint64_t peerTradeGold_ = 0;

    // Shaman totem state
    TotemSlot activeTotemSlots_[NUM_TOTEM_SLOTS];

    // Duel state
    bool pendingDuelRequest_    = false;
    uint64_t duelChallengerGuid_= 0;
    uint64_t duelFlagGuid_      = 0;
    std::string duelChallengerName_;
    uint32_t duelCountdownMs_   = 0;   // 0 = no active countdown
    std::chrono::steady_clock::time_point duelCountdownStartedAt_{};

    // ---- Guild state ----
    std::string guildName_;
    std::vector<std::string> guildRankNames_;
    GuildRosterData guildRoster_;
    GuildInfoData guildInfoData_;
    GuildQueryResponseData guildQueryData_;
    bool hasGuildRoster_ = false;
    std::unordered_map<uint32_t, std::string> guildNameCache_;  // guildId → guild name
    std::unordered_set<uint32_t> pendingGuildNameQueries_;      // in-flight guild queries
    bool pendingGuildInvite_ = false;
    std::string pendingGuildInviterName_;
    std::string pendingGuildInviteGuildName_;
    bool showPetitionDialog_ = false;
    uint32_t petitionCost_ = 0;
    uint64_t petitionNpcGuid_ = 0;
    PetitionInfo petitionInfo_;

    uint64_t activeCharacterGuid_ = 0;
    Race playerRace_ = Race::HUMAN;

    // Barber shop
    bool barberShopOpen_ = false;

    // ---- Loot ----
    bool lootWindowOpen = false;
    bool autoLoot_ = false;
    bool autoSellGrey_ = false;
    bool autoRepair_ = false;
    LootResponseData currentLoot;
    std::vector<uint64_t> masterLootCandidates_;  // from SMSG_LOOT_MASTER_LIST

    // Group loot roll state
    bool          pendingLootRollActive_ = false;
    LootRollEntry pendingLootRoll_;
    struct LocalLootState {
        LootResponseData data;
        bool moneyTaken = false;
        bool itemAutoLootSent = false;
    };
    std::unordered_map<uint64_t, LocalLootState> localLootState_;
    std::vector<PendingLootRetry> pendingGameObjectLootRetries_;
    struct PendingLootOpen {
        uint64_t guid = 0;
        float timer = 0.0f;
        uint8_t remainingAttempts = 1;
    };
    std::vector<PendingLootOpen> pendingGameObjectLootOpens_;
    // Tracks the last GO we sent CMSG_GAMEOBJ_USE to; used in handleSpellGo
    // to send CMSG_LOOT after a gather cast (mining/herbalism) completes.
    uint64_t lastInteractedGoGuid_ = 0;
    uint64_t pendingLootMoneyGuid_ = 0;
    uint32_t pendingLootMoneyAmount_ = 0;
    float pendingLootMoneyNotifyTimer_ = 0.0f;
    std::unordered_map<uint64_t, float> recentLootMoneyAnnounceCooldowns_;
    uint64_t playerMoneyCopper_ = 0;
    uint32_t playerHonorPoints_ = 0;
    uint32_t playerArenaPoints_ = 0;
    int32_t playerArmorRating_ = 0;
    int32_t playerResistances_[6] = {};  // [0]=Holy,[1]=Fire,[2]=Nature,[3]=Frost,[4]=Shadow,[5]=Arcane
    // Server-authoritative primary stats: [0]=STR [1]=AGI [2]=STA [3]=INT [4]=SPI; -1 = not received yet
    int32_t playerStats_[5] = {-1, -1, -1, -1, -1};
    // WotLK secondary combat stats (-1 = not yet received)
    int32_t playerMeleeAP_    = -1;
    int32_t playerRangedAP_   = -1;
    int32_t playerSpellDmgBonus_[7] = {-1,-1,-1,-1,-1,-1,-1}; // per school 0-6
    int32_t playerHealBonus_  = -1;
    float playerDodgePct_     = -1.0f;
    float playerParryPct_     = -1.0f;
    float playerBlockPct_     = -1.0f;
    float playerCritPct_      = -1.0f;
    float playerRangedCritPct_ = -1.0f;
    float playerSpellCritPct_[7] = {-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f};
    int32_t playerCombatRatings_[25] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};
    // Some servers/custom clients shift update field indices. We can auto-detect coinage by correlating
    // money-notify deltas with update-field diffs and then overriding UF::PLAYER_FIELD_COINAGE at runtime.
    uint32_t pendingMoneyDelta_ = 0;
    float pendingMoneyDeltaTimer_ = 0.0f;

    // Gossip
    bool gossipWindowOpen = false;
    GossipMessageData currentGossip;
    std::vector<GossipPoi> gossipPois_;

    void performGameObjectInteractionNow(uint64_t guid);

    // Quest details
    bool questDetailsOpen = false;
    std::chrono::steady_clock::time_point questDetailsOpenTime{};  // Delayed opening to allow item data to load
    QuestDetailsData currentQuestDetails;

    // Quest turn-in
    bool questRequestItemsOpen_ = false;
    QuestRequestItemsData currentQuestRequestItems_;
    uint32_t pendingTurnInQuestId_ = 0;
    uint64_t pendingTurnInNpcGuid_ = 0;
    bool pendingTurnInRewardRequest_ = false;
    // (pending quest accept timeout state lives in QuestHandler)
    bool questOfferRewardOpen_ = false;
    QuestOfferRewardData currentQuestOfferReward_;

    // Quest log
    std::vector<QuestLogEntry> questLog_;
    int selectedQuestLogIndex_ = 0;
    std::unordered_set<uint32_t> pendingQuestQueryIds_;
    std::unordered_set<uint32_t> trackedQuestIds_;
    std::unordered_set<uint32_t> mapVisibleQuestIds_;
    bool pendingLoginQuestResync_ = false;
    float pendingLoginQuestResyncTimeout_ = 0.0f;

    // Quest giver status per NPC
    std::unordered_map<uint64_t, QuestGiverStatus> npcQuestStatus_;

    // Faction hostility lookup (populated from FactionTemplate.dbc)
    std::unordered_map<uint32_t, bool> factionHostileMap_;

    // Vehicle (WotLK): non-zero when player is seated in a vehicle
    uint32_t vehicleId_ = 0;

    // Taxi / Flight Paths
    std::unordered_map<uint64_t, bool> taxiNpcHasRoutes_;  // guid -> has new/available routes
    std::unordered_map<uint32_t, TaxiNode> taxiNodes_;
    std::vector<TaxiPathEdge> taxiPathEdges_;
    std::unordered_map<uint32_t, std::vector<TaxiPathNode>> taxiPathNodes_;  // pathId -> ordered waypoints
    bool taxiDbcLoaded_ = false;
    bool taxiWindowOpen_ = false;
    ShowTaxiNodesData currentTaxiData_;
    uint64_t taxiNpcGuid_ = 0;
    bool onTaxiFlight_ = false;
    std::string taxiDestName_;
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
    std::unordered_map<uint32_t, uint32_t> taxiCostMap_; // destNodeId -> total cost in copper
    uint32_t nextMovementTimestampMs();
    void updateClientTaxi(float deltaTime);

    // Mail
    bool mailboxOpen_ = false;
    uint64_t mailboxGuid_ = 0;
    std::vector<MailMessage> mailInbox_;
    int selectedMailIndex_ = -1;
    bool showMailCompose_ = false;
    bool hasNewMail_ = false;
    std::array<MailAttachSlot, MAIL_MAX_ATTACHMENTS> mailAttachments_{};

    // Bank
    bool bankOpen_ = false;
    uint64_t bankerGuid_ = 0;
    std::array<uint64_t, 28> bankSlotGuids_{};
    std::array<uint64_t, 7> bankBagSlotGuids_{};
    int effectiveBankSlots_ = 28;     // 24 for Classic, 28 for TBC/WotLK
    int effectiveBankBagSlots_ = 7;   // 6 for Classic, 7 for TBC/WotLK

    // Guild Bank
    bool guildBankOpen_ = false;
    uint64_t guildBankerGuid_ = 0;
    GuildBankData guildBankData_;
    uint8_t guildBankActiveTab_ = 0;

    // Auction House
    bool auctionOpen_ = false;
    uint64_t auctioneerGuid_ = 0;
    uint32_t auctionHouseId_ = 0;
    AuctionListResult auctionBrowseResults_;
    AuctionListResult auctionOwnerResults_;
    AuctionListResult auctionBidderResults_;
    int auctionActiveTab_ = 0;  // 0=Browse, 1=Bids, 2=Auctions
    // Last search params for re-query (pagination, auto-refresh after bid/buyout)
    struct AuctionSearchParams {
        std::string name;
        uint8_t levelMin = 0, levelMax = 0;
        uint32_t quality = 0xFFFFFFFF;
        uint32_t itemClass = 0xFFFFFFFF;
        uint32_t itemSubClass = 0xFFFFFFFF;
        uint32_t invTypeMask = 0;
        uint8_t usableOnly = 0;
        uint32_t offset = 0;
    };
    AuctionSearchParams lastAuctionSearch_;
    // Routing: which result vector to populate from next SMSG_AUCTION_LIST_RESULT
    enum class AuctionResultTarget { BROWSE, OWNER, BIDDER };
    AuctionResultTarget pendingAuctionTarget_ = AuctionResultTarget::BROWSE;

    // Vendor
    bool vendorWindowOpen = false;
    ListInventoryData currentVendorItems;
    std::deque<BuybackItem> buybackItems_;
    std::unordered_map<uint64_t, BuybackItem> pendingSellToBuyback_;
    int pendingBuybackSlot_ = -1;
    uint32_t pendingBuybackWireSlot_ = 0;
    uint32_t pendingBuyItemId_ = 0;
    uint32_t pendingBuyItemSlot_ = 0;

    // Trainer
    bool trainerWindowOpen_ = false;
    TrainerListData currentTrainerList_;
    mutable std::unordered_map<uint32_t, SpellNameEntry> spellNameCache_;
    mutable bool spellNameCacheLoaded_ = false;

    // Title cache: maps titleBit → title string (lazy-loaded from CharTitles.dbc)
    // The strings use "%s" as a player-name placeholder (e.g. "Commander %s", "%s the Explorer").
    mutable std::unordered_map<uint32_t, std::string> titleNameCache_;
    mutable bool titleNameCacheLoaded_ = false;
    void loadTitleNameCache() const;
    // Set of title bit-indices known to the player (from SMSG_TITLE_EARNED).
    std::unordered_set<uint32_t> knownTitleBits_;
    // Currently selected title bit, or -1 for no title. Updated from PLAYER_CHOSEN_TITLE.
    int32_t chosenTitleBit_ = -1;

    // Achievement caches (lazy-loaded from Achievement.dbc on first earned event)
    std::unordered_map<uint32_t, std::string> achievementNameCache_;
    std::unordered_map<uint32_t, std::string> achievementDescCache_;
    std::unordered_map<uint32_t, uint32_t>    achievementPointsCache_;
    std::unordered_map<uint32_t, uint32_t>    achievementIconCache_;  // achievementId → SpellIcon.dbc ID
    bool achievementNameCacheLoaded_ = false;
    // Set of achievement IDs earned by the player (populated from SMSG_ALL_ACHIEVEMENT_DATA)
    std::unordered_set<uint32_t> earnedAchievements_;
    // Earn dates: achievementId → WoW PackedTime (from SMSG_ACHIEVEMENT_EARNED / SMSG_ALL_ACHIEVEMENT_DATA)
    std::unordered_map<uint32_t, uint32_t> achievementDates_;
    // Criteria progress: criteriaId → current value (from SMSG_CRITERIA_UPDATE)
    std::unordered_map<uint32_t, uint64_t> criteriaProgress_;
    void handleAllAchievementData(network::Packet& packet);

    // Per-player achievement data from SMSG_RESPOND_INSPECT_ACHIEVEMENTS
    // Key: inspected player's GUID; value: set of earned achievement IDs
    std::unordered_map<uint64_t, std::unordered_set<uint32_t>> inspectedPlayerAchievements_;

    // Area name cache (lazy-loaded from WorldMapArea.dbc; maps AreaTable ID → display name)
    mutable std::unordered_map<uint32_t, std::string> areaNameCache_;
    mutable bool areaNameCacheLoaded_ = false;
    void loadAreaNameCache() const;

    // Map metadata cache (lazy-loaded from Map.dbc).
    mutable std::unordered_map<uint32_t, std::string> mapNameCache_;
    mutable std::unordered_map<uint32_t, uint32_t> mapInstanceTypeCache_;
    mutable bool mapNameCacheLoaded_ = false;
    void loadMapNameCache() const;

    // LFG dungeon name cache (lazy-loaded from LFGDungeons.dbc; WotLK only)
    mutable std::unordered_map<uint32_t, std::string> lfgDungeonNameCache_;
    mutable bool lfgDungeonNameCacheLoaded_ = false;
    void loadLfgDungeonDbc() const;
    std::vector<TrainerTab> trainerTabs_;
    void preloadDBCCaches() const;
    void categorizeTrainerSpells();

    // Callbacks
    WorldConnectSuccessCallback onSuccess;
    WorldConnectFailureCallback onFailure;
    CharCreateCallback charCreateCallback_;
    CharDeleteCallback charDeleteCallback_;
    CharLoginFailCallback charLoginFailCallback_;
    uint8_t lastCharDeleteResult_ = 0xFF;
    bool pendingCharDeleteResponse_ = false;
    uint64_t pendingDeleteGuid_ = 0;
    float pendingDeleteTimer_ = 0.0f;
    bool pendingDeleteFallbackEnum_ = false;
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
    std::shared_ptr<WardenModule> wardenLoadedModule_; // Loaded Warden module

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

    // Async Warden response: avoids 5-second main-loop stalls from PAGE_A/PAGE_B code pattern searches
    std::future<std::vector<uint8_t>> wardenPendingEncrypted_;  // encrypted response bytes
    bool wardenResponsePending_ = false;

    // ---- RX silence detection ----
    std::chrono::steady_clock::time_point lastRxTime_{};
    bool rxSilenceLogged_ = false;
    bool rxSilence15sLogged_ = false;

    // ---- XP tracking ----
    uint32_t playerXp_ = 0;
    uint32_t playerNextLevelXp_ = 0;
    uint32_t playerRestedXp_ = 0;
    bool isResting_ = false;
    uint32_t serverPlayerLevel_ = 1;
    static uint32_t xpForLevel(uint32_t level);

    // ---- Server time tracking (for deterministic celestial/sky systems) ----
    float gameTime_ = 0.0f;       // Server game time in seconds
    float timeSpeed_ = 0.0166f;   // Time scale (default: 1 game day = 1 real hour)
    void handleLoginSetTimeSpeed(network::Packet& packet);

    // ---- Global Cooldown (GCD) ----
    float gcdTotal_ = 0.0f;
    std::chrono::steady_clock::time_point gcdStartedAt_{};

    // ---- Weather state (SMSG_WEATHER) ----
    uint32_t weatherType_ = 0;       // 0=clear, 1=rain, 2=snow, 3=storm
    float weatherIntensity_ = 0.0f;  // 0.0 to 1.0

    // ---- Light override (SMSG_OVERRIDE_LIGHT) ----
    uint32_t overrideLightId_ = 0;      // 0 = no override
    uint32_t overrideLightTransMs_ = 0;

    // ---- Player skills ----
    std::unordered_map<uint32_t, PlayerSkill> playerSkills_;
    std::unordered_map<uint32_t, std::string> skillLineNames_;
    std::unordered_map<uint32_t, uint32_t> skillLineCategories_;
    std::unordered_map<uint32_t, uint32_t> spellToSkillLine_;      // spellID -> skillLineID
    std::vector<SpellBookTab> spellBookTabs_;
    bool spellBookTabsDirty_ = true;
    bool skillLineDbcLoaded_ = false;
    bool skillLineAbilityLoaded_ = false;
    std::vector<uint32_t> playerExploredZones_ =
        std::vector<uint32_t>(PLAYER_EXPLORED_ZONES_COUNT, 0u);
    bool hasPlayerExploredZones_ = false;
    // Apply packed kill counts from player update fields to a quest entry that has
    // already had its killObjectives populated from SMSG_QUEST_QUERY_RESPONSE.
    void applyPackedKillCountsFromFields(QuestLogEntry& quest);

    NpcDeathCallback npcDeathCallback_;
    UnitRenderInstanceResolver unitRenderInstanceResolver_;
    NpcAggroCallback npcAggroCallback_;
    NpcRespawnCallback npcRespawnCallback_;
    StandStateCallback standStateCallback_;
    LogoutCompleteCallback logoutCompleteCallback_;
    AppearanceChangedCallback appearanceChangedCallback_;
    PlayerModelRebuildCallback playerModelRebuildCallback_;
    GhostStateCallback ghostStateCallback_;
    MeleeSwingCallback meleeSwingCallback_;
    FaceCameraProvider faceCameraProvider_;
    RangedWeaponSwapCallback rangedWeaponSwapCallback_;
    bool suppressMeleeSwingAnim_ = false;
    // lastMeleeSwingMs_ moved to CombatHandler
    SpellCastAnimCallback spellCastAnimCallback_;
    SpellCastFailedCallback spellCastFailedCallback_;
    UnitAnimHintCallback unitAnimHintCallback_;
    UnitMoveFlagsCallback unitMoveFlagsCallback_;
    NpcSwingCallback npcSwingCallback_;
    HitReactionCallback hitReactionCallback_;
    StunStateCallback stunStateCallback_;
    StealthStateCallback stealthStateCallback_;
    PlayerHealthCallback playerHealthCallback_;
    NpcGreetingCallback npcGreetingCallback_;
    NpcFarewellCallback npcFarewellCallback_;
    NpcVendorCallback npcVendorCallback_;
    ChargeCallback chargeCallback_;
    LevelUpCallback levelUpCallback_;
    LevelUpDeltas lastLevelUpDeltas_;
    std::vector<TempEnchantTimer> tempEnchantTimers_;
    std::vector<BookPage> bookPages_;            // pages collected for the current readable item
    OtherPlayerLevelUpCallback otherPlayerLevelUpCallback_;
    OtherPlayerMountCallback otherPlayerMountCallback_;
    AchievementEarnedCallback achievementEarnedCallback_;
    AreaDiscoveryCallback areaDiscoveryCallback_;
    QuestProgressCallback questProgressCallback_;
    MountCallback mountCallback_;
    TaxiPrecacheCallback taxiPrecacheCallback_;
    TaxiOrientationCallback taxiOrientationCallback_;
    TaxiLandingPositionCallback taxiLandingPositionCallback_;
    TaxiFlightStartCallback taxiFlightStartCallback_;
    OpenLfgCallback openLfgCallback_;
    uint32_t currentMountDisplayId_ = 0;
    uint32_t mountAuraSpellId_ = 0;       // Spell ID of the aura that caused mounting (for CMSG_CANCEL_AURA fallback)
    bool playerDead_ = false;
    bool releasedSpirit_ = false;
    uint32_t corpseMapId_ = 0;
    bool corpsePositionValid_ = false;
    float corpseX_ = 0.0f, corpseY_ = 0.0f, corpseZ_ = 0.0f;
    uint64_t corpseGuid_ = 0;
    // Absolute time (ms since epoch) when PvP corpse-reclaim delay expires.
    // 0 means no active delay (reclaim allowed immediately upon proximity).
    uint64_t corpseReclaimAvailableMs_ = 0;
    // Death Knight runes (class 6): slots 0-1=Blood, 2-3=Unholy, 4-5=Frost initially
    std::array<RuneSlot, 6> playerRunes_ = [] {
        std::array<RuneSlot, 6> r{};
        r[0].type = r[1].type = RuneType::Blood;
        r[2].type = r[3].type = RuneType::Unholy;
        r[4].type = r[5].type = RuneType::Frost;
        return r;
    }();
    uint64_t pendingSpiritHealerGuid_ = 0;
    bool resurrectPending_ = false;
    bool resurrectRequestPending_ = false;
    bool selfResAvailable_ = false;  // SMSG_PRE_RESURRECT received — Reincarnation/Twisting Nether
    // ---- Talent wipe confirm dialog ----
    // (talent wipe confirm state lives in SpellHandler)
    // ---- Pet talent respec confirm dialog ----
    // (pet unlearn confirm state lives in SpellHandler)
    bool resurrectIsSpiritHealer_ = false;  // true = SMSG_SPIRIT_HEALER_CONFIRM, false = SMSG_RESURRECT_REQUEST
    uint64_t resurrectCasterGuid_ = 0;
    std::string resurrectCasterName_;
    bool repopPending_ = false;
    uint64_t lastRepopRequestMs_ = 0;

    // ---- Completed quest IDs (SMSG_QUERY_QUESTS_COMPLETED_RESPONSE) ----
    std::unordered_set<uint32_t> completedQuests_;

    // ---- Equipment sets (SMSG_EQUIPMENT_SET_LIST) ----
    struct EquipmentSet {
        uint64_t setGuid = 0;
        uint32_t setId = 0;
        std::string name;
        std::string iconName;
        uint32_t ignoreSlotMask = 0;
        std::array<uint64_t, 19> itemGuids{};
    };
    std::vector<EquipmentSet> equipmentSets_;
    std::string pendingSaveSetName_;   // Saved between CMSG_EQUIPMENT_SET_SAVE and SMSG_EQUIPMENT_SET_SAVED
    std::string pendingSaveSetIcon_;
    std::vector<EquipmentSetInfo> equipmentSetInfo_;  // public-facing copy

    // forcedReactions_ moved to CombatHandler

    // ---- Server-triggered audio ----
    PlayMusicCallback playMusicCallback_;
    PlaySoundCallback playSoundCallback_;
    PlayPositionalSoundCallback playPositionalSoundCallback_;

    // ---- UI error frame callback ----
    UIErrorCallback uiErrorCallback_;

    // ---- Reputation change callback ----
    RepChangeCallback repChangeCallback_;
    uint32_t watchedFactionId_ = 0; // auto-set to most recently changed faction

    // ---- PvP honor credit callback ----
    PvpHonorCallback pvpHonorCallback_;

    // ---- Item loot callback ----
    ItemLootCallback itemLootCallback_;

    // ---- Loot window callback ----
    LootWindowCallback lootWindowCallback_;

    // ---- Quest completion callback ----
    QuestCompleteCallback questCompleteCallback_;

    // ---- GM Ticket state (SMSG_GMTICKET_GETTICKET / SMSG_GMTICKET_SYSTEMSTATUS) ----
    bool        gmTicketActive_    = false;  ///< True when an open ticket exists on the server
    std::string gmTicketText_;               ///< Text of the open ticket (from SMSG_GMTICKET_GETTICKET)
    float       gmTicketWaitHours_ = 0.0f;  ///< Server-estimated wait time in hours
    bool        gmSupportAvailable_ = true; ///< GM support system online (SMSG_GMTICKET_SYSTEMSTATUS)

    // ---- Battlefield Manager state (WotLK Wintergrasp / outdoor battlefields) ----
    bool        bfMgrInvitePending_ = false; ///< True when an entry/queue invite is pending acceptance
    bool        bfMgrActive_        = false; ///< True while the player is inside an outdoor battlefield
    uint32_t    bfMgrZoneId_        = 0;     ///< Zone ID of the pending/active battlefield

    // ---- WotLK Calendar: pending invite counter ----
    uint32_t    calendarPendingInvites_ = 0; ///< Unacknowledged calendar invites (SMSG_CALENDAR_SEND_NUM_PENDING)

    // ---- Spell modifiers (SMSG_SET_FLAT_SPELL_MODIFIER / SMSG_SET_PCT_SPELL_MODIFIER) ----
    // Keyed by (SpellModOp, groupIndex); cleared on logout/character change.
    std::unordered_map<SpellModKey, int32_t, SpellModKeyHash> spellFlatMods_;
    std::unordered_map<SpellModKey, int32_t, SpellModKeyHash> spellPctMods_;
};

} // namespace game
} // namespace wowee
