#pragma once

#include "game/transport_path_repository.hpp"
#include "game/transport_clock_sync.hpp"
#include "game/transport_animator.hpp"
#include <chrono>
#include <cstdint>
#include <vector>
#include <unordered_map>
#include <string>
#include <mutex>
#include <optional>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace wowee::rendering {
    class WMORenderer;
    class M2Renderer;
}

namespace wowee::pipeline {
    class AssetManager;
}

namespace wowee::game {

struct ActiveTransport {
    uint64_t guid;              // Entity GUID
    uint32_t wmoInstanceId;     // WMO renderer instance ID
    uint32_t pathId;            // Current path
    uint32_t entry = 0;         // GameObject entry (for MO_TRANSPORT path updates)
    uint32_t displayId = 0;     // GameObject display id, used for model-family-specific behavior
    glm::vec3 basePosition;     // Spawn position (base offset for path)
    glm::vec3 position;         // Current world position
    glm::quat rotation;         // Current world rotation
    glm::mat4 transform;        // Cached world transform
    glm::mat4 invTransform;     // Cached inverse for collision

    // Player attachment state
    bool playerOnBoard;
    glm::vec3 playerLocalOffset;

    // Optional deck boundaries
    glm::vec3 deckMin;
    glm::vec3 deckMax;
    bool hasDeckBounds;

    // Time-based animation (deterministic, no drift)
    uint32_t localClockMs;         // Local path time in milliseconds
    bool hasServerClock;            // Whether we've synced with server time
    int32_t serverClockOffsetMs;   // Offset: serverClock - localNow
    bool useClientAnimation;        // Use client-side path animation
    bool clientAnimationReverse;    // Run client animation in reverse along the selected path
    float serverYaw;                // Server-authoritative yaw (radians)
    bool hasServerYaw;              // Whether we've received server yaw
    float dockYaw;                  // Authored server spawn yaw used during route dwell
    bool hasDockYaw;                // Whether dockYaw was captured before client route assignment
    // The yaw this object was placed at. An elevator never turns, so for a
    // z-only path this is the heading for its whole life — there is no route
    // tangent to derive one from, and deriving anyway is how a lift ended up
    // sideways.
    float spawnYaw = 0.0f;
    bool serverYawFlipped180;       // Auto-correction when server yaw is consistently opposite movement
    int serverYawAlignmentScore;    // Hysteresis score for yaw flip detection

    double lastServerUpdate;         // Time of last server movement update
    int serverUpdateCount;          // Number of server updates received

    // Dead-reckoning from latest authoritative updates (used only when updates are sparse).
    glm::vec3 serverLinearVelocity;
    float serverAngularVelocity;
    bool hasServerVelocity;
    bool allowBootstrapVelocity;   // Disable DBC bootstrap when spawn/path mismatch is clearly invalid
    bool isM2 = false;             // True if rendered as M2 (not WMO), uses M2Renderer for transforms
    bool worldCoords = false;       // TaxiPathNode absolute-world route (client-owned WMO ship)

    // Whether the hull is currently holding at an authored dock stop, and what
    // was last pushed to its child doodads because of it. A paddlewheel that
    // keeps turning while its ship sits at the pier is the visible symptom of
    // never revisiting this: the animation was set once when the doodad spawned.
    // The count is kept because doodads stream in over several frames, so a
    // doodad attached after the last push would otherwise keep its spawn state.
    bool atDockDwell = false;
    int appliedDoodadAnim = -1;
    size_t appliedDoodadCount = 0;

    // The server's own route clock, when it publishes one (WotLK MO_TRANSPORT).
    // routePhase is the fraction of the route period the hull was at as of
    // routePhaseAtTime; routePeriodMs is that period. Preferred over the client's
    // invented period, which is what let a ferry lap its shore while the server's
    // schedule caught up.
    bool hasServerRouteClock = false;
    float routePhase = 0.0f;
    uint32_t routePeriodMs = 0;
    double routePhaseAtTime = 0.0;
};

class TransportManager {
public:
    TransportManager();
    ~TransportManager();

    // Absolute wall-clock ms. Used to seed a client-animated transport's starting phase
    // so it's deterministic across clients/restarts instead of depending on when this
    // process happened to launch or first see a given transport - see the seed call
    // sites in transport_clock_sync.cpp for the full explanation of why that matters.
    // Inline (not in transport_manager.cpp): kept dependency-free of the renderer
    // headers that file pulls in, so lightweight callers (and the transport unit
    // tests, which link transport_clock_sync.cpp/transport_animator.cpp without the
    // rest of TransportManager) can use it without linking the renderer subsystem.
    static uint64_t nowEpochMs() {
        return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());
    }

    // True for any GameObject/path identity belonging to the Deeprun Tram (both trains,
    // all 6 car entries). Shared by callers in other translation units (transport_clock_sync.cpp,
    // transport_animator.cpp, application.cpp) that need Deeprun-specific behavior.
    static bool isDeeprunTramTransport(const ActiveTransport& transport) {
        return transport.displayId == 3831u ||
               (transport.entry >= 176080u && transport.entry <= 176085u) ||
               (transport.pathId >= 176080u && transport.pathId <= 176085u);
    }

    // Single source of truth for a transport hull's fixed orientation offset:
    // its rendered facing is (direction of travel) + this constant. Every
    // orientation path funnels through here rather than re-listing ships.
    // Server-position-driven transports (trams, zeppelins) never reach it — they
    // measure the same offset live from heading-vs-velocity in updateYawAlignment().
    //
    // Measured from the art rather than guessed: every transport hull in the WoW
    // data is authored with its bow at model-space -X, so the offset is PI for
    // all of them and there are no exceptions to list.
    //
    // Two independent measurements over every .wmo under World\wmo\transports:
    // the hull tapers to a point at -X and stays blunt at +X (transportship
    // 1.0 vs 10.1 half-width at the ends, icebreaker 4.1 vs 14.8, the NE ferry,
    // the UD and pirate ships, both zeppelins and the battleships all the same
    // way); and the icebreaker is a paddle steamer whose ICEBREAKER_PADDLEWHEEL
    // doodad sits at x=+36.3 on a hull spanning -60.7..+50.1, which puts the
    // stern at +X and so the bow at -X.
    //
    // The table this replaces claimed the opposite default and then listed the
    // icebreaker and the NE ferry as the reversed ones — exactly inverted. It
    // could never have been right for everything at once, because it was fitted
    // against a facing that came from a frozen server yaw rather than from the
    // route, so what it was correcting was not the hull.
    static float transportModelBowOffset(uint32_t /*displayId*/) {
        constexpr float kBowReversed = 3.14159265358979323846f;  // PI
        return kBowReversed;
    }

    // Round a path duration to the nearest 500ms for seed-modulo purposes only. Deeprun
    // Tram cars are meant to move as one rigid train, but their individual
    // TransportAnimation.dbc entries don't all report the exact same durationMs (observed:
    // two of the six cars are ~102ms longer than the other four). nowEpochMs() % durationMs
    // takes a huge wall-clock dividend, so even a ~100ms divisor mismatch between sibling
    // cars produces effectively uncorrelated remainders - cars seeded seconds apart end up
    // on opposite sides of the loop instead of near each other ("solo cars"). Rounding both
    // divisors to the same 500ms bucket before the modulo keeps sibling cars' seeds
    // correlated by real elapsed registration time, the way they're meant to be, without
    // needing to hardcode any specific entry's duration as "the" canonical one.
    static uint32_t deeprunTramSeedDurationMs(uint32_t durationMs) {
        constexpr uint32_t kRoundTo = 500;
        return ((durationMs + kRoundTo / 2) / kRoundTo) * kRoundTo;
    }

    void setWMORenderer(rendering::WMORenderer* renderer) { wmoRenderer_ = renderer; }
    void setM2Renderer(rendering::M2Renderer* renderer) { m2Renderer_ = renderer; }

    void update(float deltaTime);
    void registerTransport(uint64_t guid,
                           uint32_t wmoInstanceId,
                           uint32_t pathId,
                           const glm::vec3& spawnWorldPos,
                           uint32_t entry = 0,
                           uint32_t displayId = 0,
                           bool isM2 = false,
                           float spawnOrientation = 0.0f);
    void unregisterTransport(uint64_t guid);
    // Logical transport GUIDs are only unique within the current map. Clear all
    // active instances on map transitions so a reused GUID cannot retain the
    // previous map's entry/path while merely rebinding to a new render model.
    void clearTransports();

    ActiveTransport* getTransport(uint64_t guid);
    // MAIN-THREAD-ONLY: this reference is not lock-protected, matching EntityManager's
    // getEntities(). Callers on another thread (e.g. a headless HTTP API thread) must
    // use snapshotTransports() instead.
    const std::unordered_map<uint64_t, ActiveTransport>& getTransports() const { return transports_; }
    // Thread-safe copy of all registered transports, safe to call from any thread.
    std::vector<ActiveTransport> snapshotTransports() const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<ActiveTransport> snapshot;
        snapshot.reserve(transports_.size());
        for (const auto& [guid, transport] : transports_) {
            snapshot.push_back(transport);
        }
        return snapshot;
    }
    glm::vec3 getPlayerWorldPosition(uint64_t transportGuid, const glm::vec3& localOffset);
    glm::vec3 serverToTransportLocal(uint64_t transportGuid,
                                     const glm::vec3& serverOffset) const;
    glm::mat4 getTransportInvTransform(uint64_t transportGuid);
    // Adopt the server's published route phase for a transport. phase is a
    // fraction in [0,1) of periodMs. Cheap and idempotent — safe to call on every
    // object update that carries the fields.
    void applyServerRouteClock(uint64_t transportGuid, float phase, uint32_t periodMs);

    // Ship machinery follows the hull: ShipMoving while under way, ShipStop when
    // holding at a dock. Cheap to call every frame — it only reaches the
    // renderer when the state or the doodad count actually changes.
    void applyDoodadMotionState(ActiveTransport& transport, bool moving);

    /// Whether this transport's hull collision has finished loading. Distinguishes
    /// "the deck query found nothing because there is no deck here" from "because
    /// the model is still uploading".
    bool isTransportCollisionReady(uint64_t transportGuid) const;

    bool isPointOnTransportDeck(uint64_t transportGuid,
                                const glm::vec3& canonicalPosition,
                                float maxFloorDelta = 1.25f) const;
    std::optional<float> getTransportDeckFloorHeight(
        uint64_t transportGuid,
        const glm::vec3& canonicalPosition) const;

    void loadPathFromNodes(uint32_t pathId, const std::vector<glm::vec3>& waypoints, bool looping = true, float speed = 18.0f);
    void setDeckBounds(uint64_t guid, const glm::vec3& min, const glm::vec3& max);

    // Load transport paths from TransportAnimation.dbc
    bool loadTransportAnimationDBC(pipeline::AssetManager* assetMgr);

    // Load transport paths from TaxiPathNode.dbc (world-coordinate paths for MO_TRANSPORT)
    bool loadTaxiPathNodeDBC(pipeline::AssetManager* assetMgr);

    // Check if a TaxiPathNode path exists for a given taxiPathId (on any map)
    bool hasTaxiPath(uint32_t taxiPathId) const;
    // Check if a TaxiPathNode segment exists for a taxiPathId on a specific map
    bool hasTaxiPathForMap(uint32_t taxiPathId, uint32_t mapId) const;

    // Assign a TaxiPathNode path to an existing transport (called when GO query response arrives).
    // mapId selects the per-map segment (cross-map boat paths only have valid world
    // geometry on the transport's current map). Returns true if the transport was updated.
    bool assignTaxiPathToTransport(uint32_t entry, uint32_t taxiPathId, uint32_t mapId);

    // Check if a path exists for a given GameObject entry
    bool hasPathForEntry(uint32_t entry) const;
    // Check if a path has meaningful XY travel (used to reject near-stationary false positives).
    bool hasUsableMovingPathForEntry(uint32_t entry, float minXYRange = 1.0f) const;

    // Infer a real moving DBC path by spawn position (for servers whose transport entry IDs
    // don't map 1:1 to TransportAnimation.dbc entry IDs).
    // Returns 0 when no suitable path match is found.
    uint32_t inferMovingPathForSpawn(const glm::vec3& spawnWorldPos, float maxDistance = 1200.0f) const;

    // Infer a DBC path by spawn position, optionally including z-only elevator paths.
    // Returns 0 when no suitable path match is found.
    uint32_t inferDbcPathForSpawn(const glm::vec3& spawnWorldPos,
                                  float maxDistance,
                                  bool allowZOnly) const;

    // Choose a deterministic fallback moving DBC path for known server transport entries/displayIds.
    // Returns 0 when no suitable moving path is available.
    uint32_t pickFallbackMovingPath(uint32_t entry, uint32_t displayId) const;

    // Update server-controlled transport position/rotation directly (bypasses path movement)
    void updateServerTransport(uint64_t guid, const glm::vec3& position, float orientation);

    // Resolve a usable path (real entry match, inferred by spawn position, remapped fallback,
    // or a stationary single-point path as last resort) and register a transport that hasn't
    // been seen before. This is the path-selection cascade every caller needs regardless of
    // whether it has a real WMO/M2 render instance for the transport (wmoInstanceId=0 is a
    // valid "no visual instance" sentinel, e.g. for a headless client that doesn't render).
    // Callers own deciding wmoInstanceId/isM2 (which requires knowing whether the transport's
    // model resolved to a WMO or M2 asset) and whether to call this at all (i.e. only when
    // getTransport(guid) is null) and whether to follow up with updateServerTransport().
    void resolveAndRegisterSpawn(uint64_t guid,
                                 uint32_t entry,
                                 uint32_t displayId,
                                 const glm::vec3& canonicalSpawnPos,
                                 uint32_t wmoInstanceId,
                                 bool isM2,
                                 bool preferServerData,
                                 float spawnOrientation = 0.0f);

    // Reconnect an existing transport to a newly spawned render instance. Servers can
    // despawn/respawn transports around visibility boundaries while the logical route
    // remains the same.
    void rebindTransportInstance(uint64_t guid, uint32_t instanceId, bool isM2, uint32_t displayId = 0);

    // Enable/disable client-side animation for transports without server updates
    void setClientSideAnimation(bool enabled) { clientSideAnimation_ = enabled; }
    bool isClientSideAnimation() const { return clientSideAnimation_; }

private:
    void updateTransportMovement(ActiveTransport& transport, float deltaTime);
    void updateTransformMatrices(ActiveTransport& transport);
    void pushTransform(ActiveTransport& transport);

    TransportPathRepository pathRepo_;
    TransportClockSync clockSync_;
    TransportAnimator animator_;
    mutable std::mutex mutex_;  // Guards transports_ map insert/erase for cross-thread snapshotTransports().
    std::unordered_map<uint64_t, ActiveTransport> transports_;
    rendering::WMORenderer* wmoRenderer_ = nullptr;
    rendering::M2Renderer* m2Renderer_ = nullptr;
    bool clientSideAnimation_ = false;  // DISABLED - use server positions instead of client prediction
    // double: float loses millisecond precision after ~4.5 hours (2^23 / 1000),
    // causing transport path interpolation to visibly jerk in long play sessions.
    double elapsedTime_ = 0.0;
};

} // namespace wowee::game
