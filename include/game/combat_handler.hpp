#pragma once

#include "game/world_packets.hpp"
#include "game/opcode_table.hpp"
#include "game/spell_defines.hpp"
#include "network/packet.hpp"
#include <chrono>
#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace wowee {
namespace game {

class GameHandler;
class Entity;

class CombatHandler {
public:
    using PacketHandler = std::function<void(network::Packet&)>;
    using DispatchTable = std::unordered_map<LogicalOpcode, PacketHandler>;

    explicit CombatHandler(GameHandler& owner);

    void registerOpcodes(DispatchTable& table);

    // --- Public API (delegated from GameHandler) ---
    void startAutoAttack(uint64_t targetGuid);
    void stopAutoAttack();
    bool isAutoAttacking() const { return autoAttacking_; }
    bool hasAutoAttackIntent() const { return autoAttackRequested_; }
    bool isInCombat() const { return autoAttacking_ || !hostileAttackers_.empty(); }
    bool isInCombatWith(uint64_t guid) const {
        return guid != 0 &&
               ((autoAttacking_ && autoAttackTarget_ == guid) ||
                (hostileAttackers_.count(guid) > 0));
    }
    uint64_t getAutoAttackTargetGuid() const { return autoAttackTarget_; }
    bool isAggressiveTowardPlayer(uint64_t guid) const { return hostileAttackers_.count(guid) > 0; }
    uint64_t getLastMeleeSwingMs() const { return lastMeleeSwingMs_; }

    // Floating combat text
    const std::vector<CombatTextEntry>& getCombatText() const { return combatText_; }
    void updateCombatText(float deltaTime);

    // Combat log (persistent rolling history)
    const std::deque<CombatLogEntry>& getCombatLog() const { return combatLog_; }
    void clearCombatLog() { combatLog_.clear(); }

    // Threat
    struct ThreatEntry {
        uint64_t victimGuid = 0;
        uint32_t threat     = 0;
    };
    const std::vector<ThreatEntry>* getThreatList(uint64_t unitGuid) const {
        auto it = threatLists_.find(unitGuid);
        return (it != threatLists_.end()) ? &it->second : nullptr;
    }

    // Hostile attacker tracking
    bool isHostileAttacker(uint64_t guid) const { return hostileAttackers_.count(guid) > 0; }
    void clearHostileAttackers() { hostileAttackers_.clear(); }

    // Forced faction reactions
    const std::unordered_map<uint32_t, uint8_t>& getForcedReactions() const { return forcedReactions_; }

    // Auto-attack timing state (read by GameHandler::update for retry/resend logic)
    bool& autoAttackOutOfRangeRef() { return autoAttackOutOfRange_; }
    float& autoAttackOutOfRangeTimeRef() { return autoAttackOutOfRangeTime_; }
    float& autoAttackRangeWarnCooldownRef() { return autoAttackRangeWarnCooldown_; }
    float& autoAttackResendTimerRef() { return autoAttackResendTimer_; }
    bool& autoAttackRetryPendingRef() { return autoAttackRetryPending_; }

    // Combat text creation (used by other handlers, e.g. spell handler for periodic damage)
    void addCombatText(CombatTextEntry::Type type, int32_t amount, uint32_t spellId,
                       bool isPlayerSource, uint8_t powerType = 0,
                       uint64_t srcGuid = 0, uint64_t dstGuid = 0);

    // Spellsteal dedup (used by aura update handler)
    bool shouldLogSpellstealAura(uint64_t casterGuid, uint64_t victimGuid, uint32_t spellId);

    // Called from GameHandler::update() each frame
    void updateAutoAttack(float deltaTime);

    // --- Targeting ---
    /// False for creature corpses that are neither lootable nor skinnable -
    /// those cannot be selected. Living units, players and non-units are always selectable.
    bool isSelectableUnit(uint64_t guid) const;
    void setTarget(uint64_t guid);
    void clearTarget();
    std::shared_ptr<Entity> getTarget() const;
    void setFocus(uint64_t guid);
    void clearFocus();
    std::shared_ptr<Entity> getFocus() const;
    void setMouseoverGuid(uint64_t guid);
    void targetLastTarget();
    void targetEnemy(bool reverse);
    void targetFriend(bool reverse);

    /// The four narrower cycles WoW binds keys to, and which answered nothing.
    ///
    /// Same walk as the two above with a tighter test: only players, or only
    /// the people in your group. Bindings.xml gives each of them a key by
    /// default, so each was a key that did nothing at all.
    void targetNearestEnemyPlayer(bool reverse);
    void targetNearestFriendPlayer(bool reverse);
    void targetNearestPartyMember(bool reverse);
    void targetNearestRaidMember(bool reverse);

    /// Back to the last hostile, or the last friendly, target.
    ///
    /// TargetLastTarget remembers one target whatever it was; these remember one
    /// of each, which is what makes them useful - a healer's last friendly
    /// target survives half a fight's worth of tab-targeting.
    void targetLastEnemy();
    void targetLastFriend();
    void tabTarget(float playerX, float playerY, float playerZ);
    void assistTarget();

    // --- PvP ---
    void togglePvp();

    // --- Death / Resurrection ---
    void releaseSpirit();
    bool canReclaimCorpse() const;
    float getCorpseReclaimDelaySec() const;
    void reclaimCorpse();
    void useSelfRes();
    void activateSpiritHealer(uint64_t npcGuid);
    void acceptResurrect();
    void declineResurrect();

    // --- XP ---
    static uint32_t killXp(uint32_t playerLevel, uint32_t victimLevel);
    void handleXpGain(network::Packet& packet);

    // State management (for resets, entity cleanup)
    void resetAllCombatState();
    void removeHostileAttacker(uint64_t guid);
    void clearCombatText();
    void removeCombatTextForGuid(uint64_t guid);

private:
    // Last tab-target press (steady-clock ms); a pause restarts the cycle
    // from the nearest enemy instead of resuming a stale rotation.
    uint64_t lastTabTargetMs_ = 0;

    // One remembered target of each kind, for targetLastEnemy/Friend. Written
    // by setTarget as the old target is let go, so they are the target before
    // this one rather than this one.
    uint64_t lastEnemyTargetGuid_ = 0;
    uint64_t lastFriendTargetGuid_ = 0;

    /// The walk both cycles do: everything alive within range that `wanted`
    /// accepts, nearest first, then step to the next one past the current
    /// target. Written out twice with two predicates before this, and the two
    /// had already drifted - one skipped the dead and the other did not, which
    /// is right both times and was not written down anywhere.
    void cycleTarget(bool reverse, const char* noneMessage,
                     const std::function<bool(uint64_t, Entity&)>& wanted);

    /// Whether a guid is in the player's group.
    bool isGroupMemberGuid(uint64_t guid) const;

    // --- Packet handlers ---
    void handleAttackStart(network::Packet& packet);
    void handleAttackStop(network::Packet& packet);
    void handleAttackerStateUpdate(network::Packet& packet);
    void handleSpellDamageLog(network::Packet& packet);
    void handleSpellHealLog(network::Packet& packet);
    void handleSetForcedReactions(network::Packet& packet);
    void handleHealthUpdate(network::Packet& packet);
    void handlePowerUpdate(network::Packet& packet);
    void handleUpdateComboPoints(network::Packet& packet);
    void handlePvpCredit(network::Packet& packet);
    void handleProcResist(network::Packet& packet);
    void handleSpellDamageShield(network::Packet& packet);
    void handleSpellOrDamageImmune(network::Packet& packet);
    void handleResistLog(network::Packet& packet);
    void handlePetTameFailure(network::Packet& packet);
    void handlePetActionFeedback(network::Packet& packet);
    void handlePetCastFailed(network::Packet& packet);
    void handlePetBroken(network::Packet& packet);
    void handlePetLearnedSpell(network::Packet& packet);
    void handlePetUnlearnedSpell(network::Packet& packet);
    void handlePetMode(network::Packet& packet);
    void handleResurrectFailed(network::Packet& packet);

    void autoTargetAttacker(uint64_t attackerGuid);

    GameHandler& owner_;

    // --- Combat state ---
    bool autoAttacking_ = false;
    bool autoAttackRequested_ = false;
    bool autoAttackRetryPending_ = false;
    uint64_t autoAttackTarget_ = 0;
    bool autoAttackOutOfRange_ = false;
    float autoAttackOutOfRangeTime_ = 0.0f;
    float autoAttackRangeWarnCooldown_ = 0.0f;
    float autoAttackResendTimer_ = 0.0f;
    std::unordered_set<uint64_t> hostileAttackers_;
    std::vector<CombatTextEntry> combatText_;
    static constexpr size_t MAX_COMBAT_LOG = 500;
    std::deque<CombatLogEntry> combatLog_;

    struct RecentSpellstealLogEntry {
        uint64_t casterGuid = 0;
        uint64_t victimGuid = 0;
        uint32_t spellId = 0;
        std::chrono::steady_clock::time_point timestamp{};
    };
    static constexpr size_t MAX_RECENT_SPELLSTEAL_LOGS = 32;
    std::deque<RecentSpellstealLogEntry> recentSpellstealLogs_;

    uint64_t lastMeleeSwingMs_ = 0;

    // unitGuid → sorted threat list (descending by threat value)
    std::unordered_map<uint64_t, std::vector<ThreatEntry>> threatLists_;
    /// Which mobs the player is currently top of the threat list for, so the
    /// aggro warning sounds on becoming that rather than on every update that
    /// says they still are. See the threat update handler.
    std::unordered_set<uint64_t> playerTopThreatOn_;
    /// Whether an aggro warning applies at all where the player is - the same
    /// question IsThreatWarningEnabled answers for the indicator, asked from
    /// C++ so the sound and the picture agree about when they are wanted.
    bool threatWarningWanted() const;

    // Forced faction reactions
    std::unordered_map<uint32_t, uint8_t> forcedReactions_;
};

} // namespace game
} // namespace wowee
