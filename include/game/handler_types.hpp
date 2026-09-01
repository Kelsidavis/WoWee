#pragma once
/**
 * handler_types.hpp - Shared struct definitions used by GameHandler and domain handlers.
 *
 * These types were previously duplicated across GameHandler, SpellHandler, SocialHandler,
 * ChatHandler, QuestHandler, and InventoryHandler.  Now they live here at namespace scope,
 * and each class provides a `using` alias for backward compatibility
 * (e.g. GameHandler::TalentEntry  ==  game::TalentEntry).
 */

#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace wowee {
namespace game {

// ---- Talent DBC data ----

struct TalentEntry {
    uint32_t talentId = 0;
    uint32_t tabId = 0;
    uint8_t row = 0;
    uint8_t column = 0;
    uint32_t rankSpells[5] = {};
    uint32_t prereqTalent[3] = {};
    uint8_t prereqRank[3] = {};
    uint8_t maxRank = 0;
};

struct TalentTabEntry {
    uint32_t tabId = 0;
    std::string name;
    uint32_t classMask = 0;
    uint8_t orderIndex = 0;
    std::string backgroundFile;
};

// ---- Spell / cast state ----

// Spell targeting classification for animation selection.
// Derived from the spell packet's targetGuid field - NOT the player's UI target.
//   DIRECTED - spell targets a specific unit (Frostbolt, Heal, Shadow Bolt)
//   OMNI     - self-cast / no explicit target (Arcane Explosion, buffs)
//   AREA     - ground-targeted AoE (Blizzard, Rain of Fire, Flamestrike)
enum class SpellCastType : uint8_t {
    DIRECTED = 0,  // Has a specific unit target
    OMNI     = 1,  // Self / no target
    AREA     = 2,  // Ground-targeted AoE
};

struct UnitCastState {
    bool          casting         = false;
    bool          isChannel       = false;
    uint32_t      spellId         = 0;
    float         timeRemaining   = 0.0f;
    float         timeTotal       = 0.0f;
    bool          interruptible   = true;
    SpellCastType castType        = SpellCastType::OMNI;
};

// ---- Equipment sets (WotLK) ----

struct EquipmentSetInfo {
    uint64_t setGuid = 0;
    uint32_t setId = 0;
    std::string name;
    std::string iconName;
};

// ---- Inspection ----

struct InspectArenaTeam {
    uint32_t    teamId         = 0;
    uint8_t     type           = 0;
    uint32_t    weekGames      = 0;
    uint32_t    weekWins       = 0;
    uint32_t    seasonGames    = 0;
    uint32_t    seasonWins     = 0;
    std::string name;
    uint32_t    personalRating = 0;
};

struct InspectResult {
    uint64_t    guid           = 0;
    std::string playerName;
    uint32_t    totalTalents   = 0;
    uint32_t    unspentTalents = 0;
    bool        hasTalentData  = false;
    bool        hasTalentTreePoints = false;
    std::array<uint32_t, 3> talentTreePoints{};
    uint8_t     talentGroups   = 0;
    uint8_t     activeTalentGroup = 0;
    std::array<uint32_t, 19> itemEntries{};
    std::array<uint16_t, 19> enchantIds{};
    std::vector<InspectArenaTeam> arenaTeams;
    /// The inspected player's own talents, as talent id -> rank, for their
    /// active spec. The inspect talent tab draws from these; without them it
    /// fell back to the viewer's own tree and attributed it to the target.
    std::unordered_map<uint32_t, uint8_t> talentRanks;
    uint8_t classId = 0;

    /// The honour tab, which arrives separately: MSG_INSPECT_HONOR_STATS is
    /// asked for by the tab when it opens and answered with its own packet.
    /// `hasHonorData` is what tells "not asked yet" from "asked and all zero",
    /// which is the difference between requesting again and drawing zeros.
    bool     hasHonorData = false;
    uint32_t honorTodayKills = 0;
    uint32_t honorYesterdayKills = 0;
    uint32_t honorTodayContribution = 0;
    uint32_t honorYesterdayContribution = 0;
    uint32_t honorLifetimeKills = 0;
    uint8_t  honorRank = 0;
};

/// An event argument that should arrive in Lua as nil rather than as text.
///
/// Every argument crosses this boundary as a string, and Lua has exactly two
/// false values - nil and false. Neither can be spelled as a string: "0" is a
/// number and true, "" is a string and true. So an event with a *false*
/// argument in front of a true one had no way to be fired correctly at all.
///
/// A start-of-heading byte, because it cannot appear in a name, a guid or any
/// game text that also crosses here.
inline constexpr const char* kEventNil = "\x01";

/// A boolean event argument: present when true, nil when false.
inline const char* eventBool(bool value) { return value ? "1" : kEventNil; }

/// One member of a chat channel, as SMSG_CHANNEL_LIST describes them.
struct ChannelMember {
    uint64_t guid = 0;
    std::string name;
    bool owner = false;
    bool moderator = false;
    bool muted = false;
};

// ---- Who ----

struct WhoEntry {
    std::string name;
    std::string guildName;
    uint32_t level    = 0;
    uint32_t classId  = 0;
    uint32_t raceId   = 0;
    uint32_t zoneId   = 0;
};

// ---- Battleground ----

struct BgQueueSlot {
    uint32_t queueSlot = 0;
    uint32_t bgTypeId = 0;
    uint8_t arenaType = 0;
    uint32_t statusId = 0;
    uint32_t inviteTimeout = 80;
    uint32_t avgWaitTimeSec = 0;
    uint32_t timeInQueueSec = 0;
    // The level range comes with the status, so a queued battleground can say
    // what it is without the available-battleground list having arrived - that
    // list only turns up at a battlemaster, and the interface asks for the range
    // every time it draws the queue.
    uint32_t minLevel = 0;
    uint32_t maxLevel = 0;
    // Both are in the status packet too. The instance number is what makes the
    // queue read "Warsong Gulch 2" rather than "Warsong Gulch", and rated is
    // what tells a rated arena from a casual one.
    uint32_t instanceId = 0;
    bool     isRated = false;
    std::chrono::steady_clock::time_point inviteReceivedTime{};
    std::string bgName;
};

struct AvailableBgInfo {
    uint32_t bgTypeId         = 0;
    bool     isRegistered     = false;
    bool     isHoliday        = false;
    uint32_t minLevel         = 0;
    uint32_t maxLevel         = 0;
    std::vector<uint32_t> instanceIds;
};

struct BgPlayerScore {
    uint64_t    guid            = 0;
    std::string name;
    uint8_t     team            = 0;
    uint32_t    killingBlows    = 0;
    uint32_t    deaths          = 0;
    uint32_t    honorableKills  = 0;
    uint32_t    bonusHonor      = 0;
    /// Both are on the wire and both were being skipped, which is why the
    /// scoreboard showed two columns of zeros.
    uint32_t    damageDone      = 0;
    uint32_t    healingDone     = 0;
    /// Whether the team above came from the packet. Arenas carry a team byte
    /// and battlegrounds do not, so a battleground row has no faction to give
    /// and saying so is better than answering zero as though it meant Horde.
    bool        hasTeam         = false;
    /// The objective values, in the order the battleground writes them. The
    /// names are not on the wire - this used to read one before each value and
    /// took every value after the first from the wrong offset.
    std::vector<std::pair<std::string, uint32_t>> bgStats;
};

struct ArenaTeamScore {
    std::string teamName;
    uint32_t    ratingChange = 0;
    uint32_t    newRating    = 0;
};

struct BgScoreboardData {
    std::vector<BgPlayerScore> players;
    bool hasWinner = false;
    uint8_t winner = 0;
    bool isArena   = false;
    ArenaTeamScore arenaTeams[2];
};

struct BgPlayerPosition {
    uint64_t guid  = 0;
    float    wowX  = 0.0f;
    float    wowY  = 0.0f;
    int      group = 0;
};

// ---- Guild event log (MSG_GUILD_EVENT_LOG_QUERY) ----
//
// Layout verified against AzerothCore's GuildEventLogQueryResults::Write
// (GuildPackets.cpp:146) rather than guessed: uint8 count, then per entry a
// type byte, the acting player's guid, a second guid only when the type is
// neither join nor leave, a rank byte only for promote and demote, and a date
// that is seconds *ago* rather than absolute.
struct GuildEventLogEntry {
    uint8_t  type = 0;
    uint64_t playerGuid = 0;
    uint64_t otherGuid = 0;      // 0 when the type carries none
    uint8_t  newRank = 0;        // only meaningful for promote/demote
    uint32_t secondsAgo = 0;
};

/// One line of a guild bank tab's log, or of the money tab's.
///
/// Types are AzerothCore's GuildBankEventLogTypes: 1 deposit item, 2 withdraw
/// item, 3 move item, 4 deposit money, 5 withdraw money, 6 repair, 7 move item
/// between tabs, 9 buy a tab. Which fields carry anything depends on which.
struct GuildBankLogEntry {
    uint8_t  type = 0;
    uint64_t playerGuid = 0;
    uint32_t itemId = 0;      ///< item types only
    uint32_t count = 0;       ///< stack size for items, copper for money
    uint8_t  otherTab = 0;    ///< a move's destination
    uint32_t secondsAgo = 0;
};

// ---- Guild petition ----

struct PetitionSignature {
    uint64_t playerGuid = 0;
    std::string playerName;
};

struct PetitionInfo {
    uint64_t petitionGuid = 0;
    uint64_t ownerGuid = 0;
    std::string guildName;
    uint32_t signatureCount = 0;
    /// How many signatures the charter needs. Only SMSG_PETITION_QUERY_RESPONSE
    /// says, so this is the fallback until that reply lands - nine is retail's
    /// guild figure, and AzerothCore's is a config the reply carries.
    uint32_t signaturesRequired = 9;
    /// Guild or arena, from the query reply. The signature frame writes a
    /// different heading and a different confirmation for each, and calling
    /// every charter a guild charter mislabels all three arena ones.
    bool isArena = false;
    std::vector<PetitionSignature> signatures;
    bool showUI = false;
};

/// One dungeon of an LFG role check: its id, and the type packed into the top
/// byte of the entry SMSG_LFG_ROLE_CHECK_UPDATE sends.
struct LfgRoleCheckDungeon {
    uint32_t dungeonId = 0;
    uint8_t typeId = 0;
};

// ---- Ready check ----

struct ReadyCheckResult {
    std::string name;
    bool ready = false;
};

// ---- Chat ----

struct ChatAutoJoin {
    bool general = true;
    bool trade = true;
    bool localDefense = true;
    bool lfg = true;
    bool local = true;
};

// ---- Quest / gossip ----

struct GossipPoi {
    float    x     = 0.0f;
    float    y     = 0.0f;
    uint32_t icon  = 0;
    uint32_t data  = 0;
    // SMSG_QUEST_POI objective index. -2 means this is a normal
    // SMSG_GOSSIP_POI; -1 identifies the quest endpoint/turn-in POI.
    int32_t  questObjectiveIndex = -2;
    std::string name;
};

// ---- Instance lockouts ----

/// What a random dungeon pays out the first time it is run each day.
struct LfgRewardItem {
    uint32_t itemId = 0;
    uint32_t displayInfoId = 0;
    uint32_t count = 0;
};
struct LfgReward {
    uint32_t dungeonId = 0;
    bool done = false;          // already claimed today
    uint32_t money = 0;
    uint32_t experience = 0;
    std::vector<LfgRewardItem> items;
};

/// One row of the dungeon-ready dialog: who the proposal is offering, and
/// whether they have answered it yet.
///
/// The server sends no names or levels here - only what each player is for and
/// what they have said - so those stay empty rather than being invented.
struct LfgProposalMember {
    uint32_t role = 0;      // LFG role mask, same bits SetLFGRoles sends
    bool isSelf = false;
    bool inDungeon = false;
    bool sameGroup = false;
    bool answered = false;
    bool accepted = false;
};

/// A mount or a critter the player knows, as the character sheet's pet tab
/// lists them. Both are spells; what separates them is what the spell does.
struct Companion {
    uint32_t spellId = 0;
    uint32_t creatureId = 0;   // EffectMiscValue - the creature summoned or ridden
    std::string name;
    bool isMount = false;
};

/// One row of LFGDungeons.dbc, as the dungeon finder needs it.
///
/// Field indices were read off the file rather than assumed: 195 records of 49
/// fields, checked against values that can be recognised - Wailing Caverns on
/// map 43, Ragefire Chasm faction 0 where everything else is -1, Karazhan in
/// the Burning Crusade raid group. The layout table only ever named ID and
/// Name, which is why the picker had nothing to list.
/// What a finished dungeon-finder run paid out, from SMSG_LFG_PLAYER_REWARD.
///
/// Kept because the alert frame reads it back rather than being handed it:
/// LFG_COMPLETION_REWARD carries no arguments, and
/// DungeonCompletionAlertFrame_ShowAlert then asks GetLFGCompletionReward for
/// all nine values and GetLFGCompletionRewardItem for each item.
struct LfgCompletionReward {
    struct Item {
        uint32_t itemId = 0;
        uint32_t displayId = 0;
        uint32_t count = 0;
    };
    uint32_t randomDungeonId = 0;  ///< the "Random ..." entry that was queued for
    uint32_t dungeonId = 0;        ///< the dungeon actually finished
    bool done = false;
    uint32_t money = 0;            ///< copper
    uint32_t xp = 0;
    std::vector<Item> items;
    bool valid = false;
};

struct LfgDungeon {
    uint32_t id = 0;
    std::string name;
    uint32_t minLevel = 0;
    uint32_t maxLevel = 0;
    uint32_t recLevel = 0;      // target level
    uint32_t minRecLevel = 0;
    uint32_t maxRecLevel = 0;
    uint32_t mapId = 0;
    uint32_t difficulty = 0;
    uint32_t typeId = 0;        // 1 dungeon, 2 raid, 4 zone, 5 heroic, 6 random
    int32_t  faction = -1;      // -1 both, 0 Horde, 1 Alliance
    std::string texture;
    uint32_t expansion = 0;
    uint32_t orderIndex = 0;
    uint32_t groupId = 0;       // 0 means it belongs under no header
    std::string description;    // the blurb the random/holiday panel prints
    bool isHoliday = false;     // group 11, the four seasonal bosses
};

/// What LFGDungeons.dbc calls each kind of row.
enum class LfgTypeId : uint32_t {
    Dungeon = 1,
    Raid    = 2,
    Zone    = 4,
    Heroic  = 5,
    Random  = 6,
};

struct InstanceLockout {
    uint32_t mapId       = 0;
    uint32_t difficulty  = 0;
    uint64_t resetTime   = 0;
    bool     locked      = false;
    bool     extended    = false;
};

// ---- LFG ----

enum class LfgState : uint8_t {
    None           = 0,
    RoleCheck      = 1,
    Queued         = 2,
    Proposal       = 3,
    Boot           = 4,
    InDungeon      = 5,
    FinishedDungeon= 6,
    RaidBrowser    = 7,
};

// ---- Arena teams ----

struct ArenaTeamStats {
    uint32_t teamId       = 0;
    uint32_t rating       = 0;
    uint32_t weekGames    = 0;
    uint32_t weekWins     = 0;
    uint32_t seasonGames  = 0;
    uint32_t seasonWins   = 0;
    uint32_t rank         = 0;
    std::string teamName;
    uint32_t teamType     = 0;
};

struct ArenaTeamMember {
    uint64_t    guid            = 0;
    std::string name;
    bool        online          = false;
    /// The three the roster carries between the name and the record. They were
    /// on the wire all along and read as part of the games played.
    bool        isCaptain       = false;
    uint8_t     level           = 0;
    uint8_t     classId         = 0;
    uint32_t    weekGames       = 0;
    uint32_t    weekWins        = 0;
    uint32_t    seasonGames     = 0;
    uint32_t    seasonWins      = 0;
    uint32_t    personalRating  = 0;
};

struct ArenaTeamRoster {
    uint32_t teamId = 0;
    std::vector<ArenaTeamMember> members;
};

// ---- Group loot roll ----

struct LootRollEntry {
    /// What the interface calls this roll, and what it passes back to
    /// RollOnLoot. Its own number rather than the loot slot: a dungeon rolls
    /// several items at once and two of them can sit in the same slot of
    /// different corpses, so the slot does not tell one roll from another.
    int      rollId        = 0;
    uint64_t objectGuid    = 0;
    uint32_t slot          = 0;
    uint32_t itemId        = 0;
    std::string itemName;
    uint8_t  itemQuality   = 0;
    uint32_t rollCountdownMs = 60000;
    uint8_t  voteMask      = 0xFF;
    std::chrono::steady_clock::time_point rollStartedAt{};

    struct PlayerRollResult {
        std::string playerName;
        uint8_t rollNum  = 0;
        uint8_t rollType = 0;
    };
    std::vector<PlayerRollResult> playerRolls;
};

} // namespace game
} // namespace wowee
