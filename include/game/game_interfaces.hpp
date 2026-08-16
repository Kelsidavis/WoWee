#pragma once

// Domain interfaces for GameHandler decomposition (Phase 1.2A).
// Each interface defines a narrow contract for a specific domain concern,
// enabling domain handlers to depend only on the state they need.

#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <array>
#include <unordered_map>

namespace wowee::network { class WorldSocket; }

namespace wowee::game {

// Forward declarations
class Entity;
class EntityManager;
enum class WorldState;
struct ContactEntry;
struct BgQueueSlot;
struct AvailableBgInfo;
struct BgScoreboardData;
struct BgPlayerPosition;
struct ArenaTeamStats;
struct ArenaTeamRoster;
struct GuildRosterData;
struct GuildInfoData;
struct GuildQueryResponseData;
struct CreatureQueryResponseData;
struct GameObjectQueryResponseData;

// ---------------------------------------------------------------------------
// IConnectionState - server connection and authentication state
// ---------------------------------------------------------------------------
class IConnectionState {
public:
    virtual ~IConnectionState() = default;

    [[nodiscard]] virtual bool isConnected() const = 0;
    [[nodiscard]] virtual bool isInWorld() const = 0;
    [[nodiscard]] virtual WorldState getState() const = 0;
    virtual network::WorldSocket* getSocket() = 0;
    [[nodiscard]] virtual const std::vector<uint8_t>& getSessionKey() const = 0;
};

// ---------------------------------------------------------------------------
// ITargetingState - target, focus, and mouseover management
// ---------------------------------------------------------------------------
class ITargetingState {
public:
    virtual ~ITargetingState() = default;

    virtual void setTarget(uint64_t guid) = 0;
    virtual void clearTarget() = 0;
    [[nodiscard]] virtual uint64_t getTargetGuid() const = 0;
    [[nodiscard]] virtual std::shared_ptr<Entity> getTarget() const = 0;
    [[nodiscard]] virtual bool hasTarget() const = 0;

    virtual void setFocus(uint64_t guid) = 0;
    virtual void clearFocus() = 0;
    [[nodiscard]] virtual uint64_t getFocusGuid() const = 0;
    [[nodiscard]] virtual bool hasFocus() const = 0;

    virtual void setMouseoverGuid(uint64_t guid) = 0;
    [[nodiscard]] virtual uint64_t getMouseoverGuid() const = 0;
};

// ---------------------------------------------------------------------------
// IEntityAccess - entity queries and name/info caching
// ---------------------------------------------------------------------------
class IEntityAccess {
public:
    virtual ~IEntityAccess() = default;

    virtual EntityManager& getEntityManager() = 0;
    [[nodiscard]] virtual const EntityManager& getEntityManager() const = 0;

    virtual void queryPlayerName(uint64_t guid) = 0;
    virtual void queryCreatureInfo(uint32_t entry, uint64_t guid) = 0;
    [[nodiscard]] virtual std::string getCachedPlayerName(uint64_t guid) const = 0;
    [[nodiscard]] virtual std::string getCachedCreatureName(uint32_t entry) const = 0;
    [[nodiscard]] virtual const std::unordered_map<uint64_t, std::string>& getPlayerNameCache() const = 0;
    [[nodiscard]] virtual const std::unordered_map<uint32_t, CreatureQueryResponseData>& getCreatureInfoCache() const = 0;
    [[nodiscard]] virtual const GameObjectQueryResponseData* getCachedGameObjectInfo(uint32_t entry) const = 0;
};

// ---------------------------------------------------------------------------
// ISocialState - friends, ignore list, contacts, guild info
// ---------------------------------------------------------------------------
class ISocialState {
public:
    virtual ~ISocialState() = default;

    virtual void addFriend(const std::string& playerName, const std::string& note = "") = 0;
    virtual void removeFriend(const std::string& playerName) = 0;
    virtual void addIgnore(const std::string& playerName) = 0;
    virtual void removeIgnore(const std::string& playerName) = 0;
    [[nodiscard]] virtual const std::unordered_map<std::string, uint64_t>& getIgnoreCache() const = 0;
    [[nodiscard]] virtual const std::vector<ContactEntry>& getContacts() const = 0;

    [[nodiscard]] virtual bool isInGuild() const = 0;
    [[nodiscard]] virtual const std::string& getGuildName() const = 0;
    [[nodiscard]] virtual const GuildRosterData& getGuildRoster() const = 0;
    [[nodiscard]] virtual bool hasGuildRoster() const = 0;
};

// ---------------------------------------------------------------------------
// IPvpState - battleground queues, arena teams, scoreboard
// ---------------------------------------------------------------------------
class IPvpState {
public:
    virtual ~IPvpState() = default;

    [[nodiscard]] virtual bool hasPendingBgInvite() const = 0;
    virtual void acceptBattlefield(uint32_t queueSlot = 0xFFFFFFFF) = 0;
    virtual void declineBattlefield(uint32_t queueSlot = 0xFFFFFFFF) = 0;
    [[nodiscard]] virtual const std::array<BgQueueSlot, 3>& getBgQueues() const = 0;
    [[nodiscard]] virtual const std::vector<AvailableBgInfo>& getAvailableBgs() const = 0;
    [[nodiscard]] virtual const BgScoreboardData* getBgScoreboard() const = 0;
    [[nodiscard]] virtual const std::vector<ArenaTeamStats>& getArenaTeamStats() const = 0;
};

} // namespace wowee::game
