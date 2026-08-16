#pragma once

#include "game/world_packets.hpp"
#include "core/coordinates.hpp"
#include "game/opcode_table.hpp"
#include "network/packet.hpp"
#include <glm/glm.hpp>
#include <chrono>
#include <deque>
#include <functional>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace wowee {
namespace game {

class GameHandler;

class MovementHandler {
public:
    using PacketHandler = std::function<void(network::Packet&)>;
    using DispatchTable = std::unordered_map<LogicalOpcode, PacketHandler>;

    explicit MovementHandler(GameHandler& owner);

    void registerOpcodes(DispatchTable& table);

    // --- Public API (delegated from GameHandler) ---

    void sendMovement(Opcode opcode);
    void setPosition(float x, float y, float z);
    void setOrientation(float orientation);
    void setMovementPitch(float radians) { movementInfo.pitch = radians; }
    void dismount();

    // Follow target (moved from GameHandler)
    void followTarget();
    void cancelFollow();
    // Per-frame movement toward the followed target's live position, called
    // from GameHandler::update() alongside the existing followRenderPos_
    // refresh. A no-op when no follow target is set. Recomputes the target
    // position fresh every call (from the live entity, not a stored point),
    // so it doesn't need the long-distance Z-interpolation guard the
    // headless client's single-shot /movement/goto needs - there's never a
    // large stale remaining distance to interpolate across.
    void updateFollowMovement(float deltaTime);

    // Area trigger detection
    void loadAreaTriggerDbc();
    void checkAreaTriggers();

    // Transport attachment
    void setTransportAttachment(uint64_t childGuid, ObjectType type, uint64_t transportGuid,
                                const glm::vec3& localOffset, bool hasLocalOrientation,
                                float localOrientation,
                                bool offsetNeedsTransportResolution = false);
    void clearTransportAttachment(uint64_t childGuid);
    void updateAttachedTransportChildren(float deltaTime);

    // Movement info accessors
    [[nodiscard]] const MovementInfo& getMovementInfo() const { return movementInfo; }
    MovementInfo& getMovementInfoMut() { return movementInfo; }

    // Speed accessors
    [[nodiscard]] float getServerRunSpeed() const { return serverRunSpeed_; }
    [[nodiscard]] float getServerWalkSpeed() const { return serverWalkSpeed_; }
    [[nodiscard]] float getServerSwimSpeed() const { return serverSwimSpeed_; }
    [[nodiscard]] float getServerSwimBackSpeed() const { return serverSwimBackSpeed_; }
    [[nodiscard]] float getServerFlightSpeed() const { return serverFlightSpeed_; }
    [[nodiscard]] float getServerFlightBackSpeed() const { return serverFlightBackSpeed_; }
    [[nodiscard]] float getServerRunBackSpeed() const { return serverRunBackSpeed_; }
    [[nodiscard]] float getServerTurnRate() const { return serverTurnRate_; }

    // CREATE_OBJECT/LIVING carries the authoritative speed table on login.
    // Apply it to the same cache used by camera movement and later force-speed
    // packets so reconnecting while mounted retains the server-provided speed.
    void applyServerMovementSpeeds(float walk, float run, float runBack,
                                   float swim, float swimBack, float flight,
                                   float flightBack, float turn, float pitch);

    // Movement flag queries
    [[nodiscard]] bool isPlayerRooted() const { return movementInfo.isPlayerRooted(); }
    [[nodiscard]] bool isGravityDisabled() const { return movementInfo.isGravityDisabled(); }
    [[nodiscard]] bool isFeatherFalling() const { return movementInfo.isFeatherFalling(); }
    [[nodiscard]] bool isWaterWalking() const { return movementInfo.isWaterWalking(); }
    [[nodiscard]] bool isPlayerFlying() const { return movementInfo.isPlayerFlying(); }
    [[nodiscard]] bool isHovering() const { return movementInfo.isHovering(); }
    [[nodiscard]] bool isSwimming() const { return movementInfo.isSwimming(); }

    // Taxi / Flight Paths
    [[nodiscard]] bool isTaxiWindowOpen() const { return taxiWindowOpen_; }
    void closeTaxi();
    void activateTaxi(uint32_t destNodeId);
    [[nodiscard]] bool isOnTaxiFlight() const { return onTaxiFlight_; }

    /// True while a locally-initiated dismount is waiting on the server to
    /// agree. The player's mount field keeps its old value for a few frames
    /// after the request, and taking that at face value re-mounts them.
    [[nodiscard]] bool isDismountPending() const { return dismountGraceRemaining_ > 0.0f; }
    void clearDismountPending() { dismountGraceRemaining_ = 0.0f; }
    [[nodiscard]] bool isTaxiMountActive() const { return taxiMountActive_; }
    [[nodiscard]] bool isTaxiActivationPending() const { return taxiActivatePending_; }
    void forceClearTaxiAndMovementState();
    [[nodiscard]] const std::string& getTaxiDestName() const { return taxiDestName_; }
    [[nodiscard]] const ShowTaxiNodesData& getTaxiData() const { return currentTaxiData_; }
    [[nodiscard]] uint32_t getTaxiCurrentNode() const { return currentTaxiData_.nearestNode; }

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

    [[nodiscard]] const std::unordered_map<uint32_t, TaxiNode>& getTaxiNodes() const { return taxiNodes_; }

    /// Drops what was read out of the taxi DBCs, so the next ask reloads them.
    ///
    /// Called when the active expansion changes. GameHandler::resetDbcCaches
    /// cleared copies of its own for this, and getTaxiNodes forwards here - so
    /// switching expansion left the previous one's flight points in place and
    /// nothing said so.
    void resetTaxiDbcCache() {
        taxiNodes_.clear();
        taxiPathEdges_.clear();
        taxiPathNodes_.clear();
        taxiDbcLoaded_ = false;
    }
    // WotLK 3.3.5a TaxiNodes.dbc has 384 entries; the known-taxi bitmask
    // is 12 × uint32 = 384 bits. Node IDs outside this range are invalid.
    static constexpr uint32_t kMaxTaxiNodeId = 384;
    [[nodiscard]] bool isKnownTaxiNode(uint32_t nodeId) const {
        if (nodeId == 0 || nodeId > kMaxTaxiNodeId) return false;
        uint32_t idx = nodeId - 1;
        return (knownTaxiMask_[idx / 32] & (1u << (idx % 32))) != 0;
    }
    [[nodiscard]] uint32_t getTaxiCostTo(uint32_t destNodeId) const;
    /// True when the taxi cost map has a route from the current node to dest.
    [[nodiscard]] bool hasTaxiRouteTo(uint32_t destNodeId) const;
    /// Node-id hop chain current → dest (inclusive); empty if unreachable.
    [[nodiscard]] std::vector<uint32_t> getTaxiRouteTo(uint32_t destNodeId) const;
    [[nodiscard]] bool taxiNpcHasRoutes(uint64_t guid) const {
        auto it = taxiNpcHasRoutes_.find(guid);
        return it != taxiNpcHasRoutes_.end() && it->second;
    }

    void updateClientTaxi(float deltaTime);
    // Ends the active client-simulated taxi flight. See the definition in
    // movement_handler.cpp for what snapToFinalWaypoint means - callers outside
    // MovementHandler (SMSG_DISMOUNT / UNIT_FIELD_MOUNTDISPLAYID handlers reacting
    // to an authoritative server completion signal) should pass false.
    void finishClientTaxiFlight(bool snapToFinalWaypoint);
    // Remember an early server-side dismount/flag clear. Some cores send the
    // completion signal well before our local spline endpoint and do not repeat
    // it, so updateClientTaxi() consumes it once the player reaches the landing
    // zone instead of waiting for the exact final waypoint.
    void deferServerTaxiCompletion();
    // Server cores can clear taxi flags/dismount before the client spline reaches
    // its final waypoint. Only treat those signals as authoritative near arrival.
    [[nodiscard]] bool isNearTaxiDestination(float maxDistance = 24.0f) const;
    uint32_t nextMovementTimestampMs();
    void sanitizeMovementForTaxi();

    // Heartbeat / movement timing (for GameHandler::update())
    float& timeSinceLastMoveHeartbeatRef() { return timeSinceLastMoveHeartbeat_; }
    [[nodiscard]] float getMoveHeartbeatInterval() const { return moveHeartbeatInterval_; }
    [[nodiscard]] bool isServerMovementAllowed() const { return serverMovementAllowed_; }
    void setServerMovementAllowed(bool v) { serverMovementAllowed_ = v; }
    uint32_t& monsterMovePacketsThisTickRef() { return monsterMovePacketsThisTick_; }
    uint32_t& monsterMovePacketsDroppedThisTickRef() { return monsterMovePacketsDroppedThisTick_; }

    // Movement clock / fall state setters (formerly accessed via friend)
    void resetMovementClock() { movementClockStart_ = std::chrono::steady_clock::now(); lastMovementTimestampMs_ = 0; }
    void setFalling(bool falling) { isFalling_ = falling; }
    void setFallStartMs(uint32_t ms) { fallStartMs_ = ms; }

    // Taxi state references for GameHandler update/processing
    bool& onTaxiFlightRef() { return onTaxiFlight_; }
    bool& taxiMountActiveRef() { return taxiMountActive_; }
    uint32_t& taxiMountDisplayIdRef() { return taxiMountDisplayId_; }
    bool& taxiActivatePendingRef() { return taxiActivatePending_; }
    float& taxiActivateTimerRef() { return taxiActivateTimer_; }
    bool& taxiClientActiveRef() { return taxiClientActive_; }
    float& taxiLandingCooldownRef() { return taxiLandingCooldown_; }
    float& taxiStartGraceRef() { return taxiStartGrace_; }
    bool& taxiRecoverPendingRef() { return taxiRecoverPending_; }
    uint32_t& taxiRecoverMapIdRef() { return taxiRecoverMapId_; }
    glm::vec3& taxiRecoverPosRef() { return taxiRecoverPos_; }
    std::unordered_map<uint64_t, bool>& taxiNpcHasRoutesRef() { return taxiNpcHasRoutes_; }
    uint32_t* knownTaxiMaskPtr() { return knownTaxiMask_; }
    bool& taxiMaskInitializedRef() { return taxiMaskInitialized_; }
    uint64_t& taxiNpcGuidRef() { return taxiNpcGuid_; }
    [[nodiscard]] uint64_t getTaxiNpcGuid() const { return taxiNpcGuid_; }

    // Other-player movement timing (for cleanup on despawn etc.)
    std::unordered_map<uint64_t, uint32_t>& otherPlayerMoveTimeMsRef() { return otherPlayerMoveTimeMs_; }
    std::unordered_map<uint64_t, float>& otherPlayerSmoothedIntervalMsRef() { return otherPlayerSmoothedIntervalMs_; }

    // Methods also called from GameHandler's registerOpcodeHandlers
    void handleCompressedMoves(network::Packet& packet);
    void handleForceMoveFlagChange(network::Packet& packet, const char* name, Opcode ackOpcode, uint32_t flag, bool set);
    void handleMoveSetCollisionHeight(network::Packet& packet);
    void applyTaxiMountForCurrentNode();

private:
    // --- Packet handlers ---
    void handleMonsterMove(network::Packet& packet);
    void handleMonsterMoveTransport(network::Packet& packet);
    void handleOtherPlayerMovement(network::Packet& packet);
    void handleMoveSetSpeed(network::Packet& packet);
    void handleForceRunSpeedChange(network::Packet& packet);
    network::Packet buildForceAck(Opcode ackOpcode, uint32_t counter);
    void handleForceSpeedChange(network::Packet& packet, const char* name, Opcode ackOpcode, float* speedStorage);
    void handleForceMoveRootState(network::Packet& packet, bool rooted);
    void handleMoveKnockBack(network::Packet& packet);
    void handleTeleportAck(network::Packet& packet);
    void handleNewWorld(network::Packet& packet);
    void handleShowTaxiNodes(network::Packet& packet);
    void handleClientControlUpdate(network::Packet& packet);
    void handleActivateTaxiReply(network::Packet& packet);
    void loadTaxiDbc();

    // --- Private helpers ---
    void buildTaxiCostMap();
    void startClientTaxiPath(const std::vector<uint32_t>& pathNodes);
    // Evaluates the same uniform Catmull-Rom curve used to render/pace taxi
    // flight motion, shared between the per-segment arc-length precomputation
    // in startClientTaxiPath() and the per-frame position update.
    static glm::vec3 evalTaxiCatmullRom(const glm::vec3& p0, const glm::vec3& p1,
                                         const glm::vec3& p2, const glm::vec3& p3, float t);
    // Commits a taxi flight already built by startClientTaxiPath(): snaps the player
    // to the path start and sets taxiClientActive_. Only called once
    // SMSG_ACTIVATETAXIREPLY confirms success - see activateTaxi()'s comment.
    void beginTaxiFlightMotion();
    bool restoreWorldTransferFallbackIfNearOrigin(const char* context);

    GameHandler& owner_;

    // --- Movement state ---
    // Reference to GameHandler's movementInfo to avoid desync
    MovementInfo& movementInfo;
    std::chrono::steady_clock::time_point movementClockStart_ = std::chrono::steady_clock::now();
    uint32_t lastMovementTimestampMs_ = 0;
    bool serverMovementAllowed_ = true;
    uint32_t monsterMovePacketsThisTick_ = 0;
    uint32_t monsterMovePacketsDroppedThisTick_ = 0;

    // Fall/jump tracking
    bool isFalling_ = false;
    uint32_t fallStartMs_ = 0;

    // Heartbeat timing
    int heartbeatLogCount_ = 0;  // periodic position audit counter
    uint32_t lastAreaTriggerId_ = 0;
    uint32_t postTransferReturnAreaTriggerId_ = 0;
    // Previous checkAreaTriggers() sample for swept-path testing (canonical
    // coords). Mounted speed covers ~3 yd per 0.25s check, enough to straddle
    // small portal boxes when only the instantaneous position is tested.
    glm::vec3 lastAreaTriggerCheckPos_{0.0f};
    uint32_t lastAreaTriggerCheckMapId_ = 0xFFFFFFFFu;
    bool lastAreaTriggerCheckValid_ = false;
    bool postTransferReturnAreaTriggerSawNear_ = false;
    bool pendingAreaTriggerDestinationValid_ = false;
    uint32_t pendingAreaTriggerDestinationMapId_ = 0;
    glm::vec3 pendingAreaTriggerDestinationServerPos_{0.0f};
    float pendingAreaTriggerDestinationServerO_ = 0.0f;
    bool worldTransferFallbackValid_ = false;
    uint32_t worldTransferFallbackMapId_ = 0;
    uint32_t worldTransferFallbackTriggerId_ = 0;
    glm::vec3 worldTransferFallbackCanonicalPos_{0.0f};
    float worldTransferFallbackCanonicalO_ = 0.0f;
    float timeSinceLastMoveHeartbeat_ = 0.0f;
    // Seconds left in which a stale non-zero mount field is not to be believed.
    // Bounded so a dismount the server refuses recovers on its own rather than
    // leaving the player permanently unable to look mounted.
    float dismountGraceRemaining_ = 0.0f;
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

    // Speed state
    float serverRunSpeed_ = 7.0f;
    float serverWalkSpeed_ = 2.5f;
    float serverRunBackSpeed_ = 4.5f;
    float serverSwimSpeed_ = 4.722f;
    float serverSwimBackSpeed_ = 2.5f;
    float serverFlightSpeed_ = 7.0f;
    float serverFlightBackSpeed_ = 4.5f;
    float serverTurnRate_ = core::coords::PI;
    float serverPitchRate_ = core::coords::PI;

    // Other-player movement smoothing
    std::unordered_map<uint64_t, uint32_t> otherPlayerMoveTimeMs_;
    std::unordered_map<uint64_t, float>    otherPlayerSmoothedIntervalMs_;

    // --- Taxi / Flight Path state ---
    std::unordered_map<uint64_t, bool> taxiNpcHasRoutes_;
    std::unordered_map<uint32_t, TaxiNode> taxiNodes_;
    std::vector<TaxiPathEdge> taxiPathEdges_;
    std::unordered_map<uint32_t, std::vector<TaxiPathNode>> taxiPathNodes_;
    bool taxiDbcLoaded_ = false;
    bool taxiWindowOpen_ = false;
    ShowTaxiNodesData currentTaxiData_;
    uint64_t taxiNpcGuid_ = 0;
    /// Triggers already reported as a near miss, so the log says it once.
    std::set<uint32_t> nearMissLogged_;
    bool onTaxiFlight_ = false;
    std::string taxiDestName_;
    // Set in activateTaxi(); used by finishClientTaxiFlight() to snap to the
    // destination TaxiNodes.dbc entry's own registered position rather than
    // taxiClientPath_'s last waypoint (from the separate TaxiPathNode.dbc
    // table) - live-confirmed the two can disagree by 100+ yards, since a
    // taxi path's own waypoint trace doesn't necessarily terminate exactly on
    // the node's registered coordinate the way GM teleports/.gps do.
    uint32_t taxiDestNodeId_ = 0;
    bool taxiMountActive_ = false;
    uint32_t taxiMountDisplayId_ = 0;
    bool taxiActivatePending_ = false;
    float taxiActivateTimer_ = 0.0f;
    // How long to wait for SMSG_ACTIVATETAXIREPLY before giving up - see
    // updateClientTaxi()'s comment on why a dropped reply needs this now.
    static constexpr float kTaxiActivateReplyTimeoutSeconds = 8.0f;
    bool taxiClientActive_ = false;
    float taxiLandingCooldown_ = 0.0f;
    float taxiStartGrace_ = 0.0f;
    size_t taxiClientIndex_ = 0;
    std::vector<glm::vec3> taxiClientPath_;
    // Arc length of the smooth Catmull-Rom curve actually rendered through each
    // consecutive pair of taxiClientPath_ points - one entry per segment, same
    // indexing as taxiClientIndex_. Pacing (how long each segment takes to
    // traverse at taxiClientSpeed_) uses this instead of the straight-line
    // chord distance between waypoints: the server's own flight spline paces
    // by the curve's real length too, and a smooth curve through a series of
    // waypoints is essentially always shorter than the straight-line polyline
    // connecting the same points (the curve "cuts" corners at turns). Using
    // chord length there made our total flight duration systematically longer
    // than the server's identical-speed (32.0f) spline - unnoticeable on a
    // short hop but live-confirmed to drift ~90 yards short of the true
    // destination on a long multi-waypoint flight (Booty Bay -> Stormwind),
    // by the time the server's own flight ends and clears UNIT_FLAG_TAXI_FLIGHT.
    // Computed once per segment when the path is built (see startClientTaxiPath()).
    std::vector<float> taxiClientSegmentArcLengths_;
    float taxiClientSpeed_ = 32.0f;
    float taxiClientSegmentProgress_ = 0.0f;
    bool taxiServerCompletionPending_ = false;
    bool taxiRecoverPending_ = false;
    uint32_t taxiRecoverMapId_ = 0;
    glm::vec3 taxiRecoverPos_{0.0f};
    uint32_t knownTaxiMask_[12] = {};
    bool taxiMaskInitialized_ = false;
    std::unordered_map<uint32_t, uint32_t> taxiCostMap_;
    // BFS predecessor per node from buildTaxiCostMap; getTaxiRouteTo walks it.
    std::unordered_map<uint32_t, uint32_t> taxiPrevMap_;

    bool followMoveMoving_ = false;  // whether MSG_MOVE_START_FORWARD has been sent for the current follow
};

} // namespace game
} // namespace wowee
