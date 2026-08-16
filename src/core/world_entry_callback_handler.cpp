#include "core/world_entry_callback_handler.hpp"
#include "core/coordinates.hpp"
#include "core/entity_spawner.hpp"
#include "core/world_loader.hpp"
#include "core/logger.hpp"
#include "rendering/renderer.hpp"
#include "rendering/animation_controller.hpp"
#include "rendering/camera_controller.hpp"
#include "rendering/terrain_manager.hpp"
#include "rendering/wmo_renderer.hpp"
#include "rendering/m2_renderer.hpp"
#include "game/game_handler.hpp"
#include "pipeline/asset_manager.hpp"

#include <cmath>
#include <cstdlib>
#include <sstream>
#include <string>

namespace wowee { namespace core {

WorldEntryCallbackHandler::WorldEntryCallbackHandler(
    rendering::Renderer& renderer,
    game::GameHandler& gameHandler,
    WorldLoader* worldLoader,
    EntitySpawner* entitySpawner,
    pipeline::AssetManager* assetManager)
    : renderer_(renderer)
    , gameHandler_(gameHandler)
    , worldLoader_(worldLoader)
    , entitySpawner_(entitySpawner)
    , assetManager_(assetManager)
{
}

// ── helpers ──────────────────────────────────────────────────────

// Sample best floor height at (x, y) from terrain, WMO, and M2 (eliminates 3x duplication)
std::optional<float> WorldEntryCallbackHandler::sampleBestFloorAt(float x, float y, float probeZ) const {
    std::optional<float> terrainFloor;
    std::optional<float> wmoFloor;
    std::optional<float> m2Floor;

    if (renderer_.getTerrainManager()) {
        terrainFloor = renderer_.getTerrainManager()->getHeightAt(x, y);
    }
    if (renderer_.getWMORenderer()) {
        wmoFloor = renderer_.getWMORenderer()->getFloorHeight(x, y, probeZ);
    }
    if (renderer_.getM2Renderer()) {
        m2Floor = renderer_.getM2Renderer()->getFloorHeight(x, y, probeZ);
    }

    std::optional<float> best;
    if (terrainFloor) best = terrainFloor;
    if (wmoFloor && (!best || *wmoFloor > *best)) best = wmoFloor;
    if (m2Floor && (!best || *m2Floor > *best)) best = m2Floor;
    return best;
}

// Clear stuck movement state on player
void WorldEntryCallbackHandler::clearStuckMovement() {
    if (renderer_.getCameraController()) {
        renderer_.getCameraController()->clearMovementInputs();
    }
    gameHandler_.forceClearTaxiAndMovementState();
    gameHandler_.sendMovement(game::Opcode::MSG_MOVE_STOP);
    gameHandler_.sendMovement(game::Opcode::MSG_MOVE_STOP_STRAFE);
    gameHandler_.sendMovement(game::Opcode::MSG_MOVE_STOP_TURN);
    gameHandler_.sendMovement(game::Opcode::MSG_MOVE_STOP_SWIM);
    gameHandler_.sendMovement(game::Opcode::MSG_MOVE_HEARTBEAT);
}

void WorldEntryCallbackHandler::clearMountForUnstuck() {
    // Unstuck changes the character's position independently of the normal
    // mount aura update path. Dismount first so the server and client agree,
    // then remove any already-orphaned local instance left by an older reset.
    const bool hasRenderedMount =
        (entitySpawner_ && entitySpawner_->getMountInstanceId() != 0) ||
        (renderer_.getAnimationController() && renderer_.getAnimationController()->isMounted());
    if (gameHandler_.isMounted() || hasRenderedMount) {
        gameHandler_.dismount();
    }
    if (entitySpawner_) {
        entitySpawner_->clearMountState();
    }
    if (auto* animation = renderer_.getAnimationController();
        animation && animation->isMounted()) {
        animation->clearMount();
    }
}

// Sync teleported render position to server
void WorldEntryCallbackHandler::syncTeleportedPositionToServer(const glm::vec3& renderPos) {
    glm::vec3 canonical = core::coords::renderToCanonical(renderPos);
    gameHandler_.setPosition(canonical.x, canonical.y, canonical.z);
    gameHandler_.sendMovement(game::Opcode::MSG_MOVE_STOP);
    gameHandler_.sendMovement(game::Opcode::MSG_MOVE_STOP_STRAFE);
    gameHandler_.sendMovement(game::Opcode::MSG_MOVE_STOP_TURN);
    gameHandler_.sendMovement(game::Opcode::MSG_MOVE_HEARTBEAT);
}

// Force server-side teleport via GM command
void WorldEntryCallbackHandler::forceServerTeleportCommand(const glm::vec3& renderPos) {
    const char* allowGmUnstuck = std::getenv("WOWEE_ALLOW_GM_UNSTUCK");
    if (!allowGmUnstuck || std::string(allowGmUnstuck) != "1") {
        LOG_WARNING("GM unstuck command suppressed. Set WOWEE_ALLOW_GM_UNSTUCK=1 to send .revive/.dismount/.go xyz.");
        return;
    }

    // Server-authoritative reset first, then teleport.
    gameHandler_.sendChatMessage(game::ChatType::SAY, ".revive", "");
    gameHandler_.sendChatMessage(game::ChatType::SAY, ".dismount", "");

    glm::vec3 canonical = core::coords::renderToCanonical(renderPos);
    glm::vec3 serverPos = core::coords::canonicalToServer(canonical);
    std::ostringstream cmd;
    cmd.setf(std::ios::fixed);
    cmd.precision(3);
    cmd << ".go xyz "
        << serverPos.x << " "
        << serverPos.y << " "
        << serverPos.z << " "
        << gameHandler_.getCurrentMapId() << " "
        << gameHandler_.getMovementInfo().orientation;
    gameHandler_.sendChatMessage(game::ChatType::SAY, cmd.str(), "");
}

// Precache tiles in a radius around a position (eliminates repeated tile-loop code)
static void precacheNearbyTiles(rendering::TerrainManager* terrainMgr,
                                const glm::vec3& renderPos, int radius) {
    if (!terrainMgr) return;
    auto [tileX, tileY] = core::coords::worldToTile(renderPos.x, renderPos.y);
    int side = 2 * radius + 1;
    std::vector<std::pair<int,int>> tiles;
    tiles.reserve(static_cast<size_t>(side) * static_cast<size_t>(side));
    for (int dy = -radius; dy <= radius; dy++)
        for (int dx = -radius; dx <= radius; dx++)
            tiles.emplace_back(tileX + dx, tileY + dy);
    terrainMgr->precacheTiles(tiles);
}

// ── callbacks ────────────────────────────────────────────────────

void WorldEntryCallbackHandler::setupCallbacks() {
    // World entry callback (online mode) - load terrain when entering world
    gameHandler_.setWorldEntryCallback([this](uint32_t mapId, float x, float y, float z, bool isInitialEntry) {
        LOG_INFO("Online world entry: mapId=", mapId, " pos=(", x, ", ", y, ", ", z, ")"
                 " initial=", isInitialEntry);
        renderer_.resetCombatVisualState();

        // Reconnect to the same map: terrain stays loaded but all online entities are stale.
        // Despawn them properly so the server's fresh CREATE_OBJECTs will re-populate the world.
        uint32_t currentLoadedMap = worldLoader_ ? worldLoader_->getLoadedMapId() : 0xFFFFFFFF;
        if (entitySpawner_ && mapId == currentLoadedMap && renderer_.getTerrainManager() && isInitialEntry) {
            LOG_INFO("Reconnect to same map ", mapId, ": clearing stale online entities (terrain preserved)");

            // Pending spawn queues and failure caches - clear so previously-failed GUIDs can retry.
            // Dead creature guids will be re-populated from fresh server state.
            entitySpawner_->clearAllQueues();

            // Properly despawn all tracked instances from the renderer
            entitySpawner_->despawnAllCreatures();
            entitySpawner_->despawnAllPlayers();
            entitySpawner_->despawnAllGameObjects();

            // Update player position and re-queue nearby tiles (same logic as teleport)
            glm::vec3 canonical = core::coords::serverToCanonical(glm::vec3(x, y, z));
            glm::vec3 renderPos = core::coords::canonicalToRender(canonical);
            renderer_.getCharacterPosition() = renderPos;
            if (renderer_.getCameraController()) {
                auto* ft = renderer_.getCameraController()->getFollowTargetMutable();
                if (ft) *ft = renderPos;
                renderer_.getCameraController()->clearMovementInputs();
                renderer_.getCameraController()->suppressMovementFor(1.0f);
                renderer_.getCameraController()->suspendGravityFor(10.0f);
            }
            worldEntryMovementGraceTimer_ = 2.0f;
            taxiLandingClampTimer_ = 0.0f;
            lastTaxiFlight_ = false;
            renderer_.getTerrainManager()->processReadyTiles();
            precacheNearbyTiles(renderer_.getTerrainManager(), renderPos, 8);
            return;
        }

        // Same-map teleport (taxi landing, GM teleport, hearthstone on same continent):
        if (mapId == currentLoadedMap && renderer_.getTerrainManager()) {
            // Check if teleport is far enough to need terrain loading (>500 render units)
            glm::vec3 oldPos = renderer_.getCharacterPosition();
            glm::vec3 canonical = core::coords::serverToCanonical(glm::vec3(x, y, z));
            glm::vec3 renderPos = core::coords::canonicalToRender(canonical);
            float teleportDistSq = glm::dot(renderPos - oldPos, renderPos - oldPos);
            bool farTeleport = (teleportDistSq > 500.0f * 500.0f);

            if (farTeleport) {
                // Far same-map teleport (hearthstone, etc.): defer full world reload
                // to next frame to avoid blocking the packet handler for 20+ seconds.
                LOG_WARNING("Far same-map teleport (dist=", std::sqrt(teleportDistSq),
                            "), deferring world reload to next frame");
                // Update position immediately so the player doesn't keep moving at old location
                renderer_.getCharacterPosition() = renderPos;
                if (renderer_.getCameraController()) {
                    auto* ft = renderer_.getCameraController()->getFollowTargetMutable();
                    if (ft) *ft = renderPos;
                    renderer_.getCameraController()->clearMovementInputs();
                    renderer_.getCameraController()->suppressMovementFor(1.0f);
                    renderer_.getCameraController()->suspendGravityFor(10.0f);
                }
                if (worldLoader_) worldLoader_->setPendingEntry(mapId, x, y, z);
                return;
            }
            LOG_INFO("Same-map teleport (map ", mapId, "), skipping full world reload");
            // canonical and renderPos already computed above for distance check
            renderer_.getCharacterPosition() = renderPos;
            if (renderer_.getCameraController()) {
                auto* ft = renderer_.getCameraController()->getFollowTargetMutable();
                if (ft) *ft = renderPos;
            }
            worldEntryMovementGraceTimer_ = 2.0f;
            taxiLandingClampTimer_ = 0.0f;
            lastTaxiFlight_ = false;
            // Stop any movement that was active before the teleport
            if (renderer_.getCameraController()) {
                renderer_.getCameraController()->clearMovementInputs();
                renderer_.getCameraController()->suppressMovementFor(0.5f);
            }
            // Kick off async upload for any tiles that finished background
            // parsing. Bounded on purpose: finalizing every ready tile here
            // stalls the main thread for seconds when many are ready, and the
            // rest finalize over subsequent frames via the terrain update loop.
            renderer_.getTerrainManager()->processReadyTiles();

            // Queue all remaining tiles within the load radius (8 tiles = 17x17)
            // at the new position. precacheTiles skips already-loaded/pending tiles,
            // so this only enqueues tiles that aren't yet in the pipeline.
            // This ensures background workers immediately start loading everything
            // visible from the new position (hearthstone may land far from old location).
            precacheNearbyTiles(renderer_.getTerrainManager(), renderPos, 8);
            return;
        }

        // If a world load is already in progress (re-entrant call from
        // gameHandler->update() processing SMSG_NEW_WORLD during warmup),
        // defer this entry. The current load will pick it up when it finishes.
        if (worldLoader_ && worldLoader_->isLoadingWorld()) {
            LOG_WARNING("World entry deferred: map ", mapId, " while loading (will process after current load)");
            worldLoader_->setPendingEntry(mapId, x, y, z);
            return;
        }

        // Full world loads are expensive and `loadOnlineWorldTerrain()` itself
        // drives `gameHandler->update()` during warmup. Queue the load here so
        // it runs after the current packet handler returns instead of recursing
        // from `SMSG_LOGIN_VERIFY_WORLD` / `SMSG_NEW_WORLD`.
        LOG_DEBUG("Queued world entry: map ", mapId, " pos=(", x, ", ", y, ", ", z, ")");
        if (worldLoader_) worldLoader_->setPendingEntry(mapId, x, y, z);
    });

    // /unstuck - nudge player forward and snap to floor at destination.
    gameHandler_.setUnstuckCallback([this]() {
        if (!renderer_.getCameraController()) return;
        clearMountForUnstuck();
        worldEntryMovementGraceTimer_ = std::max(worldEntryMovementGraceTimer_, 1.5f);
        taxiLandingClampTimer_ = 0.0f;
        lastTaxiFlight_ = false;
        clearStuckMovement();
        auto* cc = renderer_.getCameraController();
        auto* ft = cc->getFollowTargetMutable();
        if (!ft) return;

        glm::vec3 pos = *ft;

        // Always nudge forward first to escape stuck geometry (M2 models, collision seams).
        float renderYaw = gameHandler_.getMovementInfo().orientation + glm::radians(90.0f);
        pos.x += std::cos(renderYaw) * 5.0f;
        pos.y += std::sin(renderYaw) * 5.0f;

        // Sample floor at the DESTINATION position (after nudge).
        // Pick the highest floor so we snap up to WMO floors when fallen below.
        bool foundFloor = false;
        if (auto floor = sampleBestFloorAt(pos.x, pos.y, pos.z + 60.0f)) {
            pos.z = *floor + 0.2f;
            foundFloor = true;
        }

        cc->teleportTo(pos);
        if (!foundFloor) {
            cc->setGrounded(false);  // Let gravity pull player down to a surface
        }
        syncTeleportedPositionToServer(pos);
        forceServerTeleportCommand(pos);
        clearStuckMovement();
        LOG_INFO("Unstuck: nudged forward and snapped to floor");
    });

    // /unstuckgy - stronger recovery: safe/home position, then sampled floor fallback.
    gameHandler_.setUnstuckGyCallback([this]() {
        if (!renderer_.getCameraController()) return;
        clearMountForUnstuck();
        worldEntryMovementGraceTimer_ = std::max(worldEntryMovementGraceTimer_, 1.5f);
        taxiLandingClampTimer_ = 0.0f;
        lastTaxiFlight_ = false;
        clearStuckMovement();
        auto* cc = renderer_.getCameraController();
        auto* ft = cc->getFollowTargetMutable();
        if (!ft) return;

        // Try last safe position first (nearby, terrain already loaded)
        if (cc->hasLastSafePosition()) {
            glm::vec3 safePos = cc->getLastSafePosition();
            safePos.z += 5.0f;
            cc->teleportTo(safePos);
            syncTeleportedPositionToServer(safePos);
            forceServerTeleportCommand(safePos);
            clearStuckMovement();
            LOG_INFO("Unstuck: teleported to last safe position");
            return;
        }

        uint32_t bindMap = 0;
        glm::vec3 bindPos(0.0f);
        if (gameHandler_.getHomeBind(bindMap, bindPos) &&
            bindMap == gameHandler_.getCurrentMapId()) {
            bindPos.z += 2.0f;
            cc->teleportTo(bindPos);
            syncTeleportedPositionToServer(bindPos);
            forceServerTeleportCommand(bindPos);
            clearStuckMovement();
            LOG_INFO("Unstuck: teleported to home bind position");
            return;
        }

        // No safe/bind position - try current XY with a high floor probe.
        glm::vec3 pos = *ft;
        if (auto floor = sampleBestFloorAt(pos.x, pos.y, pos.z + 120.0f)) {
            pos.z = *floor + 0.5f;
            cc->teleportTo(pos);
            syncTeleportedPositionToServer(pos);
            forceServerTeleportCommand(pos);
            clearStuckMovement();
            LOG_INFO("Unstuck: teleported to sampled floor");
            return;
        }

        // Last fallback: high snap to clear deeply bad geometry.
        pos.z += 60.0f;
        cc->teleportTo(pos);
        syncTeleportedPositionToServer(pos);
        forceServerTeleportCommand(pos);
        clearStuckMovement();
        LOG_INFO("Unstuck: high fallback snap");
    });

    // /unstuckhearth - teleport to hearthstone bind point (server-synced).
    // Freezes player until terrain loads at destination to prevent falling through world.
    gameHandler_.setUnstuckHearthCallback([this]() {
        if (!renderer_.getCameraController()) return;

        uint32_t bindMap = 0;
        glm::vec3 bindPos(0.0f);
        if (!gameHandler_.getHomeBind(bindMap, bindPos)) {
            LOG_WARNING("Unstuck hearth: no bind point available");
            return;
        }

        clearMountForUnstuck();

        worldEntryMovementGraceTimer_ = 10.0f;  // long grace - terrain load check will clear it
        taxiLandingClampTimer_ = 0.0f;
        lastTaxiFlight_ = false;
        clearStuckMovement();

        auto* cc = renderer_.getCameraController();
        glm::vec3 renderPos = core::coords::canonicalToRender(bindPos);
        renderPos.z += 2.0f;

        // Freeze player in place (no gravity/movement) until terrain loads
        cc->teleportTo(renderPos);
        cc->setExternalFollow(true);
        forceServerTeleportCommand(renderPos);
        clearStuckMovement();

        // Set pending state - update loop will unfreeze once terrain is loaded
        hearthTeleportPending_ = true;
        hearthTeleportPos_ = renderPos;
        hearthTeleportTimer_ = 15.0f;  // 15s safety timeout
        LOG_INFO("Unstuck hearth: teleporting to bind point, waiting for terrain...");
    });

    // Auto-unstuck: falling for > 5 seconds = void fall, teleport to map entry
    if (renderer_.getCameraController()) {
        renderer_.getCameraController()->setAutoUnstuckCallback([this]() {
            const char* allowAutoUnstuck = std::getenv("WOWEE_ALLOW_AUTO_UNSTUCK");
            if (!allowAutoUnstuck || std::string(allowAutoUnstuck) != "1") {
                LOG_WARNING("Auto-unstuck suppressed. Set WOWEE_ALLOW_AUTO_UNSTUCK=1 to teleport to the map entry point after long falls.");
                return;
            }
            if (!renderer_.getCameraController()) return;
            clearMountForUnstuck();
            auto* cc = renderer_.getCameraController();

            // Last resort: teleport to map entry point (terrain guaranteed loaded here)
            glm::vec3 spawnPos = cc->getDefaultPosition();
            spawnPos.z += 5.0f;
            cc->teleportTo(spawnPos);
            forceServerTeleportCommand(spawnPos);
            LOG_INFO("Auto-unstuck: teleported to map entry point (server synced)");
        });
    }

    // Bind point update (innkeeper) - position stored in gameHandler->getHomeBind()
    gameHandler_.setBindPointCallback([](uint32_t mapId, float x, float y, float z) {
        LOG_INFO("Bindpoint set: mapId=", mapId, " pos=(", x, ", ", y, ", ", z, ")");
    });

    // Hearthstone preload callback: begin loading terrain at the bind point as soon as
    // the player starts casting Hearthstone.  The ~10 s cast gives enough time for
    // the background streaming workers to bring tiles into the cache so the player
    // lands on solid ground instead of falling through un-loaded terrain.
    gameHandler_.setHearthstonePreloadCallback([this](uint32_t mapId, float x, float y, float z) {
        auto* terrainMgr = renderer_.getTerrainManager();
        if (!terrainMgr || !assetManager_) return;

        // Resolve map name from the cached Map.dbc table
        std::string mapName;
        if (worldLoader_) {
            mapName = worldLoader_->getMapNameById(mapId);
        }
        if (mapName.empty()) {
            mapName = WorldLoader::mapIdToName(mapId);
        }
        if (mapName.empty()) mapName = "Azeroth";

        uint32_t currentLoadedMap = worldLoader_ ? worldLoader_->getLoadedMapId() : 0xFFFFFFFF;
        if (mapId == currentLoadedMap) {
            // Same map: pre-enqueue tiles around the bind point so workers start
            // loading them now. Uses render-space coords (canonicalToRender).
            // Use radius 4 (9x9=81 tiles) - hearthstone cast is ~10s, enough time
            // for workers to parse most of these before the player arrives.
            glm::vec3 renderPos = core::coords::canonicalToRender(glm::vec3(x, y, z));
            precacheNearbyTiles(terrainMgr, renderPos, 4);
            auto [tileX, tileY] = core::coords::worldToTile(renderPos.x, renderPos.y);
            LOG_INFO("Hearthstone preload: enqueued 81"
                     " tiles around bind point (same map) tile=[", tileX, ",", tileY, "]");
        } else {
            // Different map: warm the file cache so ADT parsing is fast when
            // loadOnlineWorldTerrain runs its blocking load loop.
            // homeBindPos_ is canonical; startWorldPreload expects server coords.
            glm::vec3 server = core::coords::canonicalToServer(glm::vec3(x, y, z));
            if (worldLoader_) {
                worldLoader_->startWorldPreload(mapId, mapName, server.x, server.y);
            }
            LOG_INFO("Hearthstone preload: started file cache warm for map '", mapName,
                     "' (id=", mapId, ")");
        }
    });
}

// ── per-frame update ─────────────────────────────────────────────

void WorldEntryCallbackHandler::update(float deltaTime) {
    // Hearth teleport: keep player frozen until terrain loads at destination
    if (hearthTeleportPending_ && renderer_.getTerrainManager()) {
        hearthTeleportTimer_ -= deltaTime;
        auto terrainH = renderer_.getTerrainManager()->getHeightAt(
            hearthTeleportPos_.x, hearthTeleportPos_.y);
        if (terrainH || hearthTeleportTimer_ <= 0.0f) {
            // Terrain loaded (or timeout) - snap to floor and release
            if (terrainH) {
                hearthTeleportPos_.z = *terrainH + 0.5f;
                renderer_.getCameraController()->teleportTo(hearthTeleportPos_);
            }
            renderer_.getCameraController()->setExternalFollow(false);
            worldEntryMovementGraceTimer_ = 1.0f;
            hearthTeleportPending_ = false;
            LOG_INFO("Unstuck hearth: terrain loaded, player released",
                     terrainH ? "" : " (timeout)");
        }
    }
}

void WorldEntryCallbackHandler::resetState() {
    hearthTeleportPending_ = false;
    hearthTeleportPos_ = glm::vec3(0.0f);
    hearthTeleportTimer_ = 0.0f;
    worldEntryMovementGraceTimer_ = 0.0f;
    lastTaxiFlight_ = false;
    taxiLandingClampTimer_ = 0.0f;
}

}} // namespace wowee::core
