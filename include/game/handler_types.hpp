#pragma once
/**
 * handler_types.hpp — Shared struct definitions used by GameHandler and domain handlers.
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
// Derived from the spell packet's targetGuid field — NOT the player's UI target.
//   DIRECTED — spell targets a specific unit (Frostbolt, Heal, Shadow Bolt)
//   OMNI     — self-cast / no explicit target (Arcane Explosion, buffs)
//   AREA     — ground-targeted AoE (Blizzard, Rain of Fire, Flamestrike)
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
    /// names are not on the wire — this used to read one before each value and
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
    uint32_t signaturesRequired = 9;
    std::vector<PetitionSignature> signatures;
    bool showUI = false;
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
