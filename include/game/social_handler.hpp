#pragma once

#include "game/world_packets.hpp"
#include "game/opcode_table.hpp"
#include "game/group_defines.hpp"
#include "game/handler_types.hpp"
#include "game/calendar_data.hpp"
#include "network/packet.hpp"
#include <array>
#include <chrono>
#include <functional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace wowee {
namespace game {

class GameHandler;

class SocialHandler {
public:
    using PacketHandler = std::function<void(network::Packet&)>;
    using DispatchTable = std::unordered_map<LogicalOpcode, PacketHandler>;

    explicit SocialHandler(GameHandler& owner);

    void registerOpcodes(DispatchTable& table);

    // ---- Structs (aliased from handler_types.hpp) ----

    using InspectArenaTeam = game::InspectArenaTeam;

    using InspectResult = game::InspectResult;

    using WhoEntry = game::WhoEntry;

    using BgQueueSlot = game::BgQueueSlot;

    using AvailableBgInfo = game::AvailableBgInfo;

    using BgPlayerScore = game::BgPlayerScore;

    using ArenaTeamScore = game::ArenaTeamScore;

    using BgScoreboardData = game::BgScoreboardData;

    using BgPlayerPosition = game::BgPlayerPosition;

    using PetitionSignature = game::PetitionSignature;

    using PetitionInfo = game::PetitionInfo;

    using ReadyCheckResult = game::ReadyCheckResult;

    using InstanceLockout = game::InstanceLockout;

    using LfgState = game::LfgState;

    using ArenaTeamStats = game::ArenaTeamStats;

    using ArenaTeamMember = game::ArenaTeamMember;

    using ArenaTeamRoster = game::ArenaTeamRoster;

    // ---- Public API ----

    // Inspection
    void inspectTarget();
    /// Inspect any player by guid; inspectTarget is this with the current target.
    void inspectUnit(uint64_t guid);
    /// Ask for the honour figures behind the inspect window's PvP tab. A
    /// separate request from the inspect itself, and answered on its own
    /// opcode, which is why the tab asks for it when it opens.
    void requestInspectHonorData(uint64_t guid);
    [[nodiscard]] const InspectResult* getInspectResult() const {
        return inspectResult_.guid ? &inspectResult_ : nullptr;
    }

    // Server info / who
    /// Ask the server for the time. `announce` prints it to chat, which is
    /// what /time wants and what the login-time query must not do.
    void queryServerTime(bool announce = false);
    /// Seconds until the daily quest reset, or 0 if the server has not said.
    /// Counted down from when the answer arrived rather than stored raw, since
    /// the reply is asked for once and read for the rest of the session.
    [[nodiscard]] uint32_t getSecondsUntilDailyReset() const;
    void requestPlayedTime();
    void queryWho(const std::string& playerName = "");
    [[nodiscard]] uint32_t getTotalTimePlayed() const { return totalTimePlayed_; }
    [[nodiscard]] uint32_t getLevelTimePlayed() const { return levelTimePlayed_; }
    [[nodiscard]] const std::vector<WhoEntry>& getWhoResults() const { return whoResults_; }
    [[nodiscard]] uint32_t getWhoOnlineCount() const { return whoOnlineCount_; }

    /// Where a /who answer goes: the panel, or the chat.
    ///
    /// SetWhoToUI is how the interface says which. FriendsFrame turns it on
    /// while its Who tab is up and off again on the way out, so a search typed
    /// into chat with the panel closed is meant to print its results there -
    /// which is what a stock client does and what this one did not, having
    /// treated the call as a no-op and always kept the rows to itself.
    void setWhoToUI(bool toUI) { whoToUI_ = toUI; }
    [[nodiscard]] bool isWhoToUI() const { return whoToUI_; }
    [[nodiscard]] std::string getWhoAreaName(uint32_t zoneId) const;

    // Social commands
    void addFriend(const std::string& playerName, const std::string& note = "");
    void removeFriend(const std::string& playerName);
    void setFriendNote(const std::string& playerName, const std::string& note);
    void addIgnore(const std::string& playerName);
    void removeIgnore(const std::string& playerName);

    // Random roll
    void randomRoll(uint32_t minRoll = 1, uint32_t maxRoll = 100);

    // Battleground
    [[nodiscard]] bool hasPendingBgInvite() const;
    void acceptBattlefield(uint32_t queueSlot = 0xFFFFFFFF);
    void declineBattlefield(uint32_t queueSlot = 0xFFFFFFFF);
    void leaveBattlefield();
    /// Ask the server which instances of one battleground are running.
    void requestBattlefieldList(uint32_t bgTypeId);
    /// Report a player in this battleground as not participating.
    void reportPvpAfk(uint64_t playerGuid);
    void joinBattlefield(uint64_t battlemasterGuid, uint32_t bgTypeId,
                         uint32_t instanceId, bool asGroup);
    [[nodiscard]] const std::array<BgQueueSlot, 3>& getBgQueues() const { return bgQueues_; }
    [[nodiscard]] const std::vector<AvailableBgInfo>& getAvailableBgs() const { return availableBgs_; }
    void requestPvpLog();
    [[nodiscard]] const BgScoreboardData* getBgScoreboard() const {
        return bgScoreboard_.players.empty() ? nullptr : &bgScoreboard_;
    }
    [[nodiscard]] const std::vector<BgPlayerPosition>& getBgPlayerPositions() const { return bgPlayerPositions_; }

    // Logout. exitAfterLogout distinguishes /quit and /exit, which leave the game
    // entirely, from /logout and /camp, which drop back to character select.
    void requestLogout(bool exitAfterLogout = false);
    void cancelLogout();
    [[nodiscard]] bool  isLoggingOut()        const { return loggingOut_; }
    [[nodiscard]] float getLogoutCountdown()  const { return logoutCountdown_; }
    [[nodiscard]] bool  isLogoutToExit()      const { return exitAfterLogout_; }

    // Guild
    void requestGuildInfo();
    void requestGuildRoster();
    /// Ask for the guild's event log. The reply is the same opcode.
    void requestGuildEventLog();
    /// Ask for one bank tab's log. Tab six is the money log, which is what the
    /// server means by GUILD_BANK_MAX_TABS.
    void requestGuildBankLog(uint8_t tab);

    /// Ask the battleground for everyone's position.
    ///
    /// MSG_BATTLEGROUND_PLAYER_POSITIONS is a request the server answers with
    /// the same opcode - the reply was already parsed here and nothing ever
    /// asked, so the list it fills stayed empty for both maps. Throttled
    /// because the interface calls it from WorldMapFrame_OnUpdate, which is
    /// every frame the map is open.
    void requestBattlefieldPositions();
    /// Ask the server to resend the friend, ignore and mute lists.
    ///
    /// The server sends them once at login and pushes a status line whenever a
    /// friend comes or goes, so this is a refresh rather than the only way to
    /// have them - but the friends panel asks for one every time it redraws,
    /// which is what ShowFriends means, and a stale online column is what
    /// happens without it.
    void requestContactList();

    /// Give or take raid assistant. CMSG_GROUP_ASSISTANT_LEADER is a guid and
    /// a flag, and AzerothCore drops it unless the sender leads the group -
    /// which is the same test the unit menu makes before offering the entry.
    void setGroupAssistant(uint64_t guid, bool apply);
    static constexpr uint8_t kGuildBankMoneyTab = 6;
    [[nodiscard]] const std::vector<GuildBankLogEntry>& getGuildBankLog(uint8_t tab) const {
        static const std::vector<GuildBankLogEntry> empty;
        return (tab < guildBankLogs_.size()) ? guildBankLogs_[tab] : empty;
    }
    [[nodiscard]] const std::vector<GuildEventLogEntry>& getGuildEventLog() const { return guildEventLog_; }
    void setGuildInfoText(const std::string& text);
    void takeInboxTextItem(uint32_t mailId);
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
    void queryGuildInfo(uint32_t guildId);
    void createGuild(const std::string& guildName);
    void addGuildRank(const std::string& rankName);
    void delGuildRank();
    void saveGuildRank(uint32_t rankId, uint32_t rights, const std::string& rankName,
                       uint32_t goldLimit, const uint32_t* tabRights, const uint32_t* tabSlots);
    void deleteGuildRank();
    void requestPetitionShowlist(uint64_t npcGuid);
    void buyPetition(uint64_t npcGuid, const std::string& guildName,
                     uint32_t clientIndex = 1);

    // Guild state accessors
    [[nodiscard]] bool isInGuild() const;
    [[nodiscard]] const std::string& getGuildName() const { return guildName_; }
    [[nodiscard]] const GuildRosterData& getGuildRoster() const { return guildRoster_; }
    [[nodiscard]] bool hasGuildRoster() const { return hasGuildRoster_; }

    /// Forget which guild the player is in, for a character that is not them.
    ///
    /// Called as a character enters the world: the name, the ranks and the
    /// roster all belong to whoever was played last, and isInGuild() reads the
    /// name.
    void clearGuildMembership() {
        guildName_.clear();
        guildRankNames_.clear();
        guildRoster_ = GuildRosterData{};
        hasGuildRoster_ = false;
        guildQueryData_ = GuildQueryResponseData{};
    }
    [[nodiscard]] const std::vector<std::string>& getGuildRankNames() const { return guildRankNames_; }
    /// The rights bitmask of the player's own guild rank, or 0 when it is not
    /// known. The roster carries one per rank and the member row names which
    /// rank each member holds; both were parsed and stored and neither was
    /// being read.
    /// Where the player sits in the guild, as an index into the rank list.
    /// 0xFFFFFFFF when the roster has not arrived or the player is not in it -
    /// distinct from rank zero, which is the guild master.
    [[nodiscard]] uint32_t getPlayerGuildRankIndex() const;
    [[nodiscard]] uint32_t getPlayerGuildRankRights() const;
    [[nodiscard]] bool hasPendingGuildInvite() const { return pendingGuildInvite_; }
    [[nodiscard]] const std::string& getPendingGuildInviterName() const { return pendingGuildInviterName_; }
    [[nodiscard]] const std::string& getPendingGuildInviteGuildName() const { return pendingGuildInviteGuildName_; }
    [[nodiscard]] const GuildInfoData& getGuildInfoData() const { return guildInfoData_; }
    [[nodiscard]] const GuildQueryResponseData& getGuildQueryData() const { return guildQueryData_; }
    [[nodiscard]] bool hasGuildInfoData() const { return guildInfoData_.isValid(); }

    // Petition
    [[nodiscard]] bool hasPetitionShowlist() const { return showPetitionDialog_; }
    void clearPetitionDialog() { showPetitionDialog_ = false; }
    /// Shuts the vendor session and tells the interface. Both windows read a
    /// different thing, so closing one has to close the other.
    void closePetitionVendor();
    [[nodiscard]] bool isGuildRegistrar() const { return petitionIsGuildCharter_; }
    /// The charters on offer, in the order the vendor listed them - one from a
    /// guild registrar, three from an arena registrar for the two, three and
    /// five person teams.
    [[nodiscard]] const std::vector<PetitionShowlistData::Charter>& getPetitionCharters() const {
        return petitionCharters_;
    }
    [[nodiscard]] uint32_t getPetitionCost() const { return petitionCost_; }
    [[nodiscard]] uint64_t getPetitionNpcGuid() const { return petitionNpcGuid_; }
    [[nodiscard]] const PetitionInfo& getPetitionInfo() const { return petitionInfo_; }
    [[nodiscard]] bool hasPetitionSignaturesUI() const { return petitionInfo_.showUI; }
    void clearPetitionSignaturesUI() { petitionInfo_.showUI = false; }
    void signPetition(uint64_t petitionGuid);
    void turnInPetition(uint64_t petitionGuid);
    /// Offer the charter to another player so they can sign it.
    void offerPetition(uint64_t petitionGuid, uint64_t targetGuid);

    // Guild name lookup
    const std::string& lookupGuildName(uint32_t guildId);
    [[nodiscard]] uint32_t getEntityGuildId(uint64_t guid) const;

    // Ready check
    void initiateReadyCheck();
    void respondToReadyCheck(bool ready);
    [[nodiscard]] bool hasPendingReadyCheck() const { return pendingReadyCheck_; }
    void dismissReadyCheck() { pendingReadyCheck_ = false; }
    [[nodiscard]] const std::string& getReadyCheckInitiator() const { return readyCheckInitiator_; }
    [[nodiscard]] const std::vector<ReadyCheckResult>& getReadyCheckResults() const { return readyCheckResults_; }

    // Duel
    void acceptDuel();
    void forfeitDuel();
    void proposeDuel(uint64_t targetGuid);
    void reportPlayer(uint64_t targetGuid, const std::string& reason);
    [[nodiscard]] bool hasPendingDuelRequest() const { return pendingDuelRequest_; }
    [[nodiscard]] const std::string& getDuelChallengerName() const { return duelChallengerName_; }
    [[nodiscard]] float getDuelCountdownRemaining() const {
        if (duelCountdownMs_ == 0) return 0.0f;
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - duelCountdownStartedAt_).count();
        float rem = (static_cast<float>(duelCountdownMs_) - static_cast<float>(elapsed)) / 1000.0f;
        return rem > 0.0f ? rem : 0.0f;
    }

    // Party/Raid
    void inviteToGroup(const std::string& playerName);
    void acceptGroupInvite();
    void declineGroupInvite();

    // ---- Arena team invitations ----
    // Accept and decline carry no payload: the server answers from the invite
    // it is already holding against this character.
    void acceptArenaTeamInvite();
    void declineArenaTeamInvite();
    void arenaTeamInvite(uint32_t teamId, const std::string& name);
    void arenaTeamLeave(uint32_t teamId);
    void arenaTeamRemove(uint32_t teamId, const std::string& name);
    void arenaTeamSetLeader(uint32_t teamId, const std::string& name);
    void disbandArenaTeam(uint32_t teamId);
    /// Report a mail as spam. Type 0 of CMSG_COMPLAIN; the three values after
    /// the sender are a zero, the mail's own id, and another zero.
    void reportMailSpam(uint64_t senderGuid, uint32_t mailId);
    void leaveGroup();
    void convertToRaid();
    void sendSetLootMethod(uint32_t method, uint32_t threshold, uint64_t masterLooterGuid);
    [[nodiscard]] bool isInGroup() const { return !partyData.isEmpty(); }
    [[nodiscard]] const GroupListData& getPartyData() const { return partyData; }
    [[nodiscard]] bool hasPendingGroupInvite() const { return pendingGroupInvite; }
    [[nodiscard]] const std::string& getPendingInviterName() const { return pendingInviterName; }
    void setGuildBankTabText(uint8_t tab, const std::string& text);
    void channelModeration(Opcode op, const std::string& channelName,
                           const std::string& targetName,
                           bool allowEmptyTarget = false);
    void promoteToLeader(uint64_t guid);
    void uninvitePlayer(const std::string& playerName);
    void setLootMethod(uint8_t method, uint64_t masterGuid, uint8_t threshold);
    void setPartyAssignment(uint8_t assignment, uint64_t guid, bool apply);
    void setRaidSubgroup(const std::string& playerName, uint8_t group);
    void swapRaidSubgroup(const std::string& firstName, const std::string& secondName);
    void leaveParty();
    void setMainTank(uint64_t targetGuid);
    void setMainAssist(uint64_t targetGuid);
    void clearMainTank();
    void clearMainAssist();
    void setRaidMark(uint64_t guid, uint8_t icon);
    // Apply a mark without the server round-trip, used when the player has no
    // group (the server would drop the request). Grouped marking stays
    // server-authoritative so every member sees the same icons.
    void setRaidMarkLocally(uint64_t guid, uint8_t icon);
    void requestRaidInfo();
    void setSavedInstanceExtend(uint32_t mapId, uint32_t difficulty, bool extend);
    /// The roles last offered to the dungeon finder. lfgSetRoles used to send
    /// them and keep nothing, so the panel reading its own checkboxes back, and
    /// the ready dialog naming the role it found you a group for, had no source.
    [[nodiscard]] uint8_t getLfgOfferedRoles() const { return lfgOfferedRoles_; }
    [[nodiscard]] int32_t  getLfgWaitTank() const   { return lfgWaitTank_; }
    [[nodiscard]] int32_t  getLfgWaitHealer() const { return lfgWaitHealer_; }
    [[nodiscard]] int32_t  getLfgWaitDps() const    { return lfgWaitDps_; }
    [[nodiscard]] uint8_t  getLfgNeedTank() const   { return lfgNeedTank_; }
    [[nodiscard]] uint8_t  getLfgNeedHealer() const { return lfgNeedHealer_; }
    [[nodiscard]] uint8_t  getLfgNeedDps() const    { return lfgNeedDps_; }
    [[nodiscard]] uint64_t getLfgBootVictimGuid() const { return lfgBootVictimGuid_; }
    [[nodiscard]] const std::unordered_map<uint32_t, uint32_t>& getLfgLocks() const { return lfgLocks_; }
    [[nodiscard]] const std::vector<LfgReward>& getLfgRewards() const { return lfgRewards_; }
    void handleLfgPlayerInfo(network::Packet& packet);
    [[nodiscard]] const std::vector<LfgProposalMember>& getLfgProposalMembers() const {
        return lfgProposalMembers_;
    }

    // Instance lockouts
    [[nodiscard]] const std::vector<InstanceLockout>& getInstanceLockouts() const { return instanceLockouts_; }

    // Instance difficulty
    [[nodiscard]] uint32_t getInstanceDifficulty() const { return instanceDifficulty_; }
    [[nodiscard]] bool isInstanceHeroic() const { return instanceIsHeroic_; }
    [[nodiscard]] bool isInInstance() const { return inInstance_; }

    // Minimap ping
    void sendMinimapPing(float wowX, float wowY);

    // Summon request
    void handleSummonRequest(network::Packet& packet);
    /// CMSG_SUMMON_RESPONSE: the summoner's guid, then yes or no.
    void sendSummonResponse(bool accept);
    void acceptSummon();
    void declineSummon();

    // Battlefield Manager
    void acceptBfMgrInvite();
    void declineBfMgrInvite();
    void respondBfMgrQueueInvite(uint32_t battleId, bool accept);
    void requestBfMgrExit(uint32_t battleId);

    // Calendar
    void requestCalendar();
    /// Invite someone: CMSG_CALENDAR_EVENT_INVITE. A pre-invite belongs to an
    /// event that has not been created yet, and carries no event id.
    void inviteToCalendarEvent(uint64_t eventId, uint64_t inviteId,
                               const std::string& name, bool isPreInvite,
                               bool isGuildEvent);
    /// Set another invitee's status, or their moderator rank. Both act on
    /// someone else; the player's own answer is respondToCalendarInvite.
    void setCalendarInviteStatus(uint64_t inviteeGuid, uint64_t eventId,
                                 uint64_t inviteId, uint8_t status);
    void setCalendarInviteModerator(uint64_t inviteeGuid, uint64_t eventId,
                                    uint64_t inviteId, uint8_t rank);
    /// Edit an existing event: CMSG_CALENDAR_UPDATE_EVENT.
    void updateCalendarEvent(uint64_t eventId, uint64_t inviteId,
                             const CalendarEventDraft& draft);
    /// Delete one: CMSG_CALENDAR_REMOVE_EVENT.
    void removeCalendarEvent(uint64_t eventId, uint64_t inviteId);
    /// Invite the guild by filter: CMSG_CALENDAR_GUILD_FILTER.
    void massInviteGuildToCalendarEvent(uint32_t minLevel, uint32_t maxLevel,
                                        uint32_t minRank);
    /// Ask for one event's detail: CMSG_CALENDAR_GET_EVENT.
    void requestCalendarEvent(uint64_t eventId);
    /// Create an event: CMSG_CALENDAR_ADD_EVENT.
    void createCalendarEvent(const CalendarEventDraft& draft);
    /// Answer an invitation: CMSG_CALENDAR_EVENT_RSVP with a
    /// CalendarInviteStatus (1 accepted, 2 declined, 8 tentative, 9 removed).
    void respondToCalendarInvite(uint64_t eventId, uint64_t inviteId,
                                 uint32_t status);

    // ---- Methods moved from GameHandler ----
    void sendSetDifficulty(uint32_t difficulty, bool raid = false);
    void toggleHelm();
    void toggleCloak();
    void setStandState(uint8_t standState);
    void sendAlterAppearance(uint32_t hairStyleEntry, uint32_t hairColor,
                             uint32_t facialHairEntry, uint32_t skinColorEntry);
    void deleteGmTicket();
    void requestGmTicket();
    /// Ask whether the GM ticket queue is accepting tickets.
    void requestGmSystemStatus();

    // Utility methods for delegation from GameHandler
    void updateLogoutCountdown(float deltaTime);
    void resetTransferState();
    GroupListData& mutablePartyData() { return partyData; }
    InspectResult& mutableInspectResult() { return inspectResult_; }
    void setRaidTargetGuid(uint8_t icon, uint64_t guid) {
        if (icon < kRaidMarkCount) raidTargetGuids_[icon] = guid;
    }
    void setEncounterUnitGuid(uint32_t slot, uint64_t guid) {
        if (slot < kMaxEncounterSlots) encounterUnitGuids_[slot] = guid;
    }

    // The bind-or-leave question the server asks on entering an instance that
    // is already under way. It carries a countdown, and the answer is the
    // player's: accept and be saved, decline and go to the graveyard. Nothing
    // happens until one of the two is sent, so this is held rather than
    // answered here.
    struct InstanceLockPrompt {
        bool active = false;
        float secondsLeft = 0.0f;
        bool previouslySaved = false;
        uint32_t completedEncounterMask = 0;
    };
    [[nodiscard]] const InstanceLockPrompt& getInstanceLockPrompt() const { return instanceLock_; }
    void respondInstanceLock(bool accept);

    // Encounter unit tracking
    static constexpr uint32_t kMaxEncounterSlots = 5;
    [[nodiscard]] uint64_t getEncounterUnitGuid(uint32_t slot) const {
        return (slot < kMaxEncounterSlots) ? encounterUnitGuids_[slot] : 0;
    }

    // Raid target markers (0-7: Star, Circle, Diamond, Triangle, Moon, Square, Cross, Skull)
    static constexpr uint32_t kRaidMarkCount = game::kRaidMarkCount;
    [[nodiscard]] uint64_t getRaidMarkGuid(uint32_t icon) const {
        return (icon < kRaidMarkCount) ? raidTargetGuids_[icon] : 0;
    }
    [[nodiscard]] uint8_t getEntityRaidMark(uint64_t guid) const {
        if (guid == 0) return 0xFF;
        for (uint32_t i = 0; i < kRaidMarkCount; ++i)
            if (raidTargetGuids_[i] == guid) return static_cast<uint8_t>(i);
        return 0xFF;
    }

    // LFG / Dungeon Finder
    /// Queue for every dungeon the player ticked. CMSG_LFG_JOIN carries a
    /// list, and the server queues for all of them at once.
    void lfgJoin(const std::vector<uint32_t>& dungeonIds, uint8_t roles);
    void lfgLeave();
    void lfgSetRoles(uint8_t roles);
    void lfgAcceptProposal(uint32_t proposalId, bool accept);
    void lfgSetBootVote(bool vote);
    void lfgTeleport(bool toLfgDungeon = true);
    [[nodiscard]] LfgState getLfgState()           const { return lfgState_; }
    /// What the last finished dungeon paid out, for GetLFGCompletionReward.
    [[nodiscard]] const LfgCompletionReward& getLfgCompletionReward() const { return lfgCompletionReward_; }
    [[nodiscard]] bool isLfgQueued()               const { return lfgState_ == LfgState::Queued; }
    [[nodiscard]] bool isLfgInDungeon()            const { return lfgState_ == LfgState::InDungeon; }
    [[nodiscard]] uint32_t getLfgDungeonId()       const { return lfgDungeonId_; }
    [[nodiscard]] std::string getCurrentLfgDungeonName() const;
    [[nodiscard]] uint32_t getLfgProposalId()      const { return lfgProposalId_; }
    [[nodiscard]] int32_t  getLfgAvgWaitSec()      const { return lfgAvgWaitSec_; }
    /// This player's own estimate, in seconds, or -1 while the server has none.
    [[nodiscard]] int32_t  getLfgMyWaitSec()       const { return lfgMyWaitSec_; }
    /// How long this player has been in the queue, in seconds.
    [[nodiscard]] uint32_t getLfgQueuedSeconds()   const { return lfgQueuedSeconds_; }
    [[nodiscard]] uint32_t getLfgBootVotes()       const { return lfgBootVotes_; }
    [[nodiscard]] uint32_t getLfgBootTotal()       const { return lfgBootTotal_; }
    [[nodiscard]] uint32_t getLfgBootTimeLeft()    const { return lfgBootTimeLeft_; }
    [[nodiscard]] uint32_t getLfgBootNeeded()      const { return lfgBootNeeded_; }
    [[nodiscard]] const std::string& getLfgBootTargetName() const { return lfgBootTargetName_; }
    [[nodiscard]] const std::string& getLfgBootReason()     const { return lfgBootReason_; }
    [[nodiscard]] const std::vector<LfgRoleCheckDungeon>& getLfgRoleCheckDungeons() const {
        return lfgRoleCheckDungeons_;
    }
    [[nodiscard]] uint8_t getLfgRoleCheckMembers() const { return lfgRoleCheckMembers_; }
    [[nodiscard]] bool isLfgBootInProgress() const { return lfgBootInProgress_; }
    [[nodiscard]] bool hasLfgBootVoted()     const { return lfgBootDidVote_; }
    [[nodiscard]] bool getLfgBootMyVote()    const { return lfgBootMyVote_; }

    // Arena
    [[nodiscard]] const std::vector<ArenaTeamStats>& getArenaTeamStats() const { return arenaTeamStats_; }
    void requestArenaTeamRoster(uint32_t teamId);
    [[nodiscard]] const ArenaTeamRoster* getArenaTeamRoster(uint32_t teamId) const {
        for (const auto& r : arenaTeamRosters_)
            if (r.teamId == teamId) return &r;
        return nullptr;
    }

private:
    // ---- Packet handlers ----
    void handleInspectResults(network::Packet& packet);
    void handleGuildBankLog(network::Packet& packet);
    void sendBfMgrResponse(Opcode op, uint32_t battleId, bool accept, bool withFlag);
    void handleQueryTimeResponse(network::Packet& packet);
    void handlePlayedTime(network::Packet& packet);
    void handleWho(network::Packet& packet);
    void handleFriendList(network::Packet& packet);
    void handleContactList(network::Packet& packet);
    void handleFriendStatus(network::Packet& packet);
    void handleRandomRoll(network::Packet& packet);
    void handleLogoutResponse(network::Packet& packet);
    void handleLogoutComplete(network::Packet& packet);
    void handleGroupInvite(network::Packet& packet);
    void handleGroupDecline(network::Packet& packet);
    void handleGroupList(network::Packet& packet);
    void handleGroupUninvite(network::Packet& packet);
    void handlePartyCommandResult(network::Packet& packet);
    void handlePartyMemberStats(network::Packet& packet, bool isFull);
    void handleGuildInfo(network::Packet& packet);
    void handleGuildRoster(network::Packet& packet);
    void handleGuildQueryResponse(network::Packet& packet);
    void handleGuildEvent(network::Packet& packet);
    // Updates a roster member's cached online flag; returns true only on a real change so
    // the login-time SIGNED_ON flood (state unchanged) is suppressed from guild chat.
    bool guildMemberOnlineTransition(const std::string& name, bool nowOnline);
    void handleGuildInvite(network::Packet& packet);
    void handleGuildCommandResult(network::Packet& packet);
    void handlePetitionShowlist(network::Packet& packet);
    void handlePetitionQueryResponse(network::Packet& packet);
    void handleGuildEventLog(network::Packet& packet);
    void handlePetitionShowSignatures(network::Packet& packet);
    void handlePetitionSignResults(network::Packet& packet);
    void handleTurnInPetitionResults(network::Packet& packet);
    void handleBattlefieldStatus(network::Packet& packet);
    void handleBattlefieldList(network::Packet& packet);
    void handleRaidInstanceInfo(network::Packet& packet);
    void handleInstanceDifficulty(network::Packet& packet);
    void handleDuelRequested(network::Packet& packet);
    void handleDuelComplete(network::Packet& packet);
    void handleDuelWinner(network::Packet& packet);
    void handleLfgJoinResult(network::Packet& packet);
    void handleLfgQueueStatus(network::Packet& packet);
    void handleLfgProposalUpdate(network::Packet& packet);
    void handleLfgRoleCheckUpdate(network::Packet& packet);
    void handleLfgUpdatePlayer(network::Packet& packet);
    void handleLfgUpdateParty(network::Packet& packet);
    void readLfgDungeonList(network::Packet& packet);
    void applyLfgUpdate(uint8_t updateType, bool extraInfo, bool queued,
                        bool inDungeon);
    void handleLfgPlayerReward(network::Packet& packet);
    void handleLfgBootProposalUpdate(network::Packet& packet);
    void handleLfgTeleportDenied(network::Packet& packet);
    void handleArenaTeamCommandResult(network::Packet& packet);
    void handleArenaTeamQueryResponse(network::Packet& packet);
    void handleArenaTeamRoster(network::Packet& packet);
    void handleArenaTeamInvite(network::Packet& packet);
    void handleArenaTeamEvent(network::Packet& packet);
    void handleArenaTeamStats(network::Packet& packet);
    void handleArenaError(network::Packet& packet);
    void handlePvpLogData(network::Packet& packet);
    void handleInitializeFactions(network::Packet& packet);
    void handleSetFactionStanding(network::Packet& packet);
    void handleSetFactionAtWar(network::Packet& packet);
    void handleSetFactionVisible(network::Packet& packet);
    void handleGroupSetLeader(network::Packet& packet);
    void handleTalentsInfo(network::Packet& packet);
    void loadGuildNameCache();
    void saveGuildNameCache() const;
    void rememberGuildName(uint32_t guildId, const std::string& guildName);

    GameHandler& owner_;

    // ---- State ----

    // Inspect
    InspectResult inspectResult_;

    // Logout
    bool  loggingOut_        = false;
    bool  exitAfterLogout_   = false;
    float logoutCountdown_   = 0.0f;

    // Time played
    uint32_t totalTimePlayed_ = 0;
    /// SMSG_QUERY_TIME_RESPONSE's second word and when it landed. Zero means
    /// no answer yet, which is different from "the reset is now".
    uint32_t dailyResetOffset_ = 0;
    time_t   dailyResetReceivedAt_ = 0;
    bool     announceServerTime_ = false;
    uint32_t levelTimePlayed_ = 0;

    // Who results
    std::vector<WhoEntry> whoResults_;
    uint32_t whoOnlineCount_ = 0;
    /// Off until the interface asks otherwise, which is a stock client's
    /// default: with no panel open the results belong in the chat.
    bool whoToUI_ = false;

    // Duel
    bool pendingDuelRequest_    = false;
    uint64_t duelChallengerGuid_= 0;
    std::string duelChallengerName_;
    uint32_t duelCountdownMs_   = 0;
    std::chrono::steady_clock::time_point duelCountdownStartedAt_{};

    // Guild
    std::string guildName_;
    std::vector<std::string> guildRankNames_;
    GuildRosterData guildRoster_;
    GuildInfoData guildInfoData_;
    GuildQueryResponseData guildQueryData_;
    bool hasGuildRoster_ = false;
    std::unordered_map<uint32_t, std::string> guildNameCache_;
    std::unordered_map<uint32_t, std::chrono::steady_clock::time_point> pendingGuildNameQueries_;
    bool pendingGuildInvite_ = false;
    std::string pendingGuildInviterName_;
    std::string pendingGuildInviteGuildName_;
    bool showPetitionDialog_ = false;
    /// Which of the two registrar panels is open - the offer says, not the
    /// opcode, and closing has to name the same one that opened.
    bool petitionIsGuildCharter_ = false;
    std::vector<PetitionShowlistData::Charter> petitionCharters_;
    uint32_t petitionCost_ = 0;
    uint64_t petitionNpcGuid_ = 0;
    PetitionInfo petitionInfo_;

    // Group
    GroupListData partyData;
    bool pendingGroupInvite = false;
    std::string pendingInviterName;

    // Ready check
    bool        pendingReadyCheck_       = false;
    uint32_t    readyCheckReadyCount_    = 0;
    uint32_t    readyCheckNotReadyCount_ = 0;
    std::string readyCheckInitiator_;
    std::vector<ReadyCheckResult> readyCheckResults_;

    // Instance
    std::vector<InstanceLockout> instanceLockouts_;
    std::vector<GuildEventLogEntry> guildEventLog_;
    /// Six item tabs and the money log after them.
    std::array<std::vector<GuildBankLogEntry>, 7> guildBankLogs_;
    uint32_t instanceDifficulty_ = 0;
    bool instanceIsHeroic_ = false;
    bool inInstance_ = false;

    // Raid marks
    std::array<uint64_t, kRaidMarkCount> raidTargetGuids_ = {};

    // Encounter units
    std::array<uint64_t, kMaxEncounterSlots> encounterUnitGuids_ = {};
    InstanceLockPrompt instanceLock_;

    // Arena
    std::vector<ArenaTeamStats>  arenaTeamStats_;
    std::vector<ArenaTeamRoster> arenaTeamRosters_;

    // Battleground
    std::array<BgQueueSlot, 3> bgQueues_{};
    std::vector<AvailableBgInfo> availableBgs_;
    BgScoreboardData bgScoreboard_;
    std::vector<BgPlayerPosition> bgPlayerPositions_;
    std::chrono::steady_clock::time_point lastBgPositionRequest_{};
    std::chrono::steady_clock::time_point lastContactListRequest_{};

    // LFG / Dungeon Finder
    LfgState lfgState_        = LfgState::None;
    LfgCompletionReward lfgCompletionReward_;
    uint32_t lfgDungeonId_    = 0;
    uint32_t lfgProposalId_   = 0;
    /// The proposal the ready dialog has already been opened for. The server
    /// resends the update as each member answers, and opening again would
    /// reset the countdown and the ticks beside every name.
    uint32_t shownProposalId_ = 0;
    uint8_t  lfgOfferedRoles_ = 0;
    // The queue's own numbers, which never arrived while the length check was
    // longer than the packet.
    int32_t  lfgWaitTank_ = -1;
    int32_t  lfgWaitHealer_ = -1;
    int32_t  lfgWaitDps_ = -1;
    uint8_t  lfgNeedTank_ = 0;
    uint8_t  lfgNeedHealer_ = 0;
    uint8_t  lfgNeedDps_ = 0;
    uint64_t lfgBootVictimGuid_ = 0;
    /// Why the server will not let this character queue for a dungeon, by
    /// dungeon id. Empty means nothing is locked, which is also what it meant
    /// while SMSG_LFG_PLAYER_INFO was being skipped - the difference is that
    /// now it is an answer rather than an absence.
    std::unordered_map<uint32_t, uint32_t> lfgLocks_;
    std::vector<LfgReward> lfgRewards_;
    /// The group a proposal is offering, in the order the server lists it.
    std::vector<LfgProposalMember> lfgProposalMembers_;
    int32_t  lfgAvgWaitSec_   = -1;
    int32_t  lfgMyWaitSec_    = -1;
    uint32_t lfgQueuedSeconds_= 0;
    uint32_t lfgBootVotes_    = 0;
    uint32_t lfgBootTotal_    = 0;
    uint32_t lfgBootTimeLeft_ = 0;
    uint32_t lfgBootNeeded_   = 0;
    /// The three flags the proposal opens with. Read from the packet and
    /// dropped until 2026-08-06; the vote dialog branches on all three.
    /// What the role check is for, and how many are answering it. Read off
    /// SMSG_LFG_ROLE_CHECK_UPDATE, which used to stop at the count.
    std::vector<LfgRoleCheckDungeon> lfgRoleCheckDungeons_;
    uint8_t lfgRoleCheckMembers_ = 0;
    bool lfgBootInProgress_ = false;
    bool lfgBootDidVote_    = false;
    bool lfgBootMyVote_     = false;
    std::string lfgBootTargetName_;
    std::string lfgBootReason_;
};

} // namespace game
} // namespace wowee
