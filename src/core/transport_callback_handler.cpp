#include "core/transport_callback_handler.hpp"
#include "core/appearance_composer.hpp"
#include "core/entity_spawner.hpp"
#include "core/world_loader.hpp"
#include "core/coordinates.hpp"
#include "core/logger.hpp"
#include "rendering/renderer.hpp"
#include "rendering/animation_controller.hpp"
#include "rendering/character_renderer.hpp"
#include "rendering/camera_controller.hpp"
#include "rendering/terrain_manager.hpp"
#include "rendering/m2_renderer.hpp"
#include "game/game_handler.hpp"
#include "game/transport_manager.hpp"

#include <set>

namespace wowee { namespace core {

TransportCallbackHandler::TransportCallbackHandler(
    EntitySpawner& entitySpawner,
    rendering::Renderer& renderer,
    game::GameHandler& gameHandler,
    WorldLoader* worldLoader,
    AppearanceComposer* appearanceComposer)
    : entitySpawner_(entitySpawner)
    , renderer_(renderer)
    , gameHandler_(gameHandler)
    , worldLoader_(worldLoader)
    , appearanceComposer_(appearanceComposer)
{
}

void TransportCallbackHandler::setupCallbacks() {
    // Mount callback (online mode) - defer heavy model load to next frame
    gameHandler_.setMountCallback([this](uint32_t mountDisplayId) {
        if (mountDisplayId == 0) {
            // Dismount is instant (no loading needed)
            if (renderer_.getCharacterRenderer() && entitySpawner_.getMountInstanceId() != 0) {
                renderer_.getCharacterRenderer()->removeInstance(entitySpawner_.getMountInstanceId());
                entitySpawner_.clearMountState();
            }
            entitySpawner_.setMountDisplayId(0);
            if (auto* ac = renderer_.getAnimationController()) ac->clearMount();
            LOG_INFO("Dismounted");
            return;
        }
        // Queue the mount for processing in the next update() frame
        entitySpawner_.setMountDisplayId(mountDisplayId);

        // Mounting stows drawn weapons, matching the original client.
        if (appearanceComposer_ && !appearanceComposer_->isWeaponsSheathed()) {
            appearanceComposer_->setWeaponsSheathed(true);
            appearanceComposer_->loadEquippedWeapons();
        }
    });

    // Taxi precache callback - preload terrain tiles along flight path
    gameHandler_.setTaxiPrecacheCallback([this](const std::vector<glm::vec3>& path) {
        if (!renderer_.getTerrainManager()) return;

        std::set<std::pair<int, int>> uniqueTiles;

        // Sample waypoints along path and gather tiles.
        // Denser sampling + neighbor coverage reduces in-flight stream spikes.
        const size_t stride = 2;
        for (size_t i = 0; i < path.size(); i += stride) {
            const auto& waypoint = path[i];
            glm::vec3 renderPos = core::coords::canonicalToRender(waypoint);
            int tileX = static_cast<int>(32 - (renderPos.x / 533.33333f));
            int tileY = static_cast<int>(32 - (renderPos.y / 533.33333f));

            if (tileX >= 0 && tileX <= 63 && tileY >= 0 && tileY <= 63) {
                for (int dx = -1; dx <= 1; ++dx) {
                    for (int dy = -1; dy <= 1; ++dy) {
                        int nx = tileX + dx;
                        int ny = tileY + dy;
                        if (nx >= 0 && nx <= 63 && ny >= 0 && ny <= 63) {
                            uniqueTiles.insert({nx, ny});
                        }
                    }
                }
            }
        }
        // Ensure final destination tile is included.
        if (!path.empty()) {
            glm::vec3 renderPos = core::coords::canonicalToRender(path.back());
            int tileX = static_cast<int>(32 - (renderPos.x / 533.33333f));
            int tileY = static_cast<int>(32 - (renderPos.y / 533.33333f));
            if (tileX >= 0 && tileX <= 63 && tileY >= 0 && tileY <= 63) {
                for (int dx = -1; dx <= 1; ++dx) {
                    for (int dy = -1; dy <= 1; ++dy) {
                        int nx = tileX + dx;
                        int ny = tileY + dy;
                        if (nx >= 0 && nx <= 63 && ny >= 0 && ny <= 63) {
                            uniqueTiles.insert({nx, ny});
                        }
                    }
                }
            }
        }

        std::vector<std::pair<int, int>> tilesToLoad(uniqueTiles.begin(), uniqueTiles.end());
        if (tilesToLoad.size() > 512) {
            tilesToLoad.resize(512);
        }
        LOG_INFO("Precaching ", tilesToLoad.size(), " tiles for taxi route");
        renderer_.getTerrainManager()->precacheTiles(tilesToLoad);
    });

    // Taxi orientation callback - update mount rotation during flight
    gameHandler_.setTaxiOrientationCallback([this](float yaw, float pitch, float roll) {
        if (renderer_.getCameraController()) {
            // Taxi callback now provides render-space yaw directly.
            float yawDegrees = glm::degrees(yaw);
            renderer_.getCameraController()->setFacingYaw(yawDegrees);
            renderer_.setCharacterYaw(yawDegrees);
            // Set mount pitch and roll for realistic flight animation
            if (auto* ac = renderer_.getAnimationController()) ac->setMountPitchRoll(pitch, roll);
        }
    });

    // Taxi landing position correction callback - see its declaration comment in
    // game_handler.hpp for why this exists (Application's own per-frame render
    // sync stops as soon as onTaxi goes false, which happens before this fires).
    gameHandler_.setPlayerPositionCorrectionCallback([this](float x, float y, float z) {
        glm::vec3 renderPos = core::coords::canonicalToRender(glm::vec3(x, y, z));
        renderer_.getCharacterPosition() = renderPos;
        if (renderer_.getCameraController()) {
            glm::vec3* followTarget = renderer_.getCameraController()->getFollowTargetMutable();
            if (followTarget) {
                *followTarget = renderPos;
            }
        }
    });

    // Taxi flight start callback - keep non-blocking to avoid hitching at takeoff.
    gameHandler_.setTaxiFlightStartCallback([this]() {
        if (renderer_.getTerrainManager() && renderer_.getM2Renderer()) {
            LOG_INFO("Taxi flight start: incremental terrain/M2 streaming active");
            uint32_t m2Count = renderer_.getM2Renderer()->getModelCount();
            uint32_t instCount = renderer_.getM2Renderer()->getInstanceCount();
            LOG_INFO("Current M2 VRAM state: ", m2Count, " models (", instCount, " instances)");
        }
    });

    // Transport spawn callback (online mode) - register transports with TransportManager
    gameHandler_.setTransportSpawnCallback([this](uint64_t guid, uint32_t entry, uint32_t displayId, float x, float y, float z, float orientation) {
        // Get the GameObject instance now so late queue processing can rely on stable IDs.
        auto& goInstances2 = entitySpawner_.getGameObjectInstances();
        auto it = goInstances2.find(guid);
        if (it == goInstances2.end()) {
            LOG_WARNING("Transport spawn callback: GameObject instance not found for GUID 0x", std::hex, guid, std::dec);
            return;
        }

        auto pendingIt = entitySpawner_.hasTransportRegistrationPending(guid);
        if (pendingIt) {
            entitySpawner_.updateTransportRegistration(guid, displayId, x, y, z, orientation);
        } else {
            entitySpawner_.queueTransportRegistration(guid, entry, displayId, x, y, z, orientation);
        }
    });

    // Transport move callback (online mode) - update transport gameobject positions
    gameHandler_.setTransportMoveCallback([this](uint64_t guid, float x, float y, float z, float orientation) {
        LOG_DEBUG("Transport move callback: GUID=0x", std::hex, guid, std::dec,
                 " pos=(", x, ", ", y, ", ", z, ") orientation=", orientation);

        auto* transportManager = gameHandler_.getTransportManager();
        if (!transportManager) {
            LOG_WARNING("Transport move callback: TransportManager is null!");
            return;
        }

        if (entitySpawner_.hasTransportRegistrationPending(guid)) {
            entitySpawner_.setTransportPendingMove(guid, x, y, z, orientation);
            LOG_DEBUG("Queued transport move for pending registration GUID=0x", std::hex, guid, std::dec);
            return;
        }

        // Check if transport exists - if not, treat this as a late spawn (reconnection/server restart)
        if (!transportManager->getTransport(guid)) {
            LOG_DEBUG("Received position update for unregistered transport 0x", std::hex, guid, std::dec,
                     " - auto-spawning from position update");

            // Get transport info from entity manager
            auto entity = gameHandler_.getEntityManager().getEntity(guid);
            if (entity && entity->getType() == game::ObjectType::GAMEOBJECT) {
                auto go = std::static_pointer_cast<game::GameObject>(entity);
                uint32_t entry = go->getEntry();
                uint32_t displayId = go->getDisplayId();

                // Find the WMO instance for this transport (should exist from earlier GameObject spawn)
                auto& goInstances3 = entitySpawner_.getGameObjectInstances();
                auto it = goInstances3.find(guid);
                if (it != goInstances3.end()) {
                    uint32_t wmoInstanceId = it->second.instanceId;
                    const bool preferServerData = gameHandler_.hasServerTransportUpdate(guid);
                    // Coordinates are already canonical (converted in game_handler.cpp)
                    glm::vec3 canonicalSpawnPos(x, y, z);
                    const bool isM2Transport = !it->second.isWmo;
                    transportManager->resolveAndRegisterSpawn(guid, entry, displayId, canonicalSpawnPos,
                                                              wmoInstanceId, isM2Transport, preferServerData,
                                                              orientation);
                } else {
                    entitySpawner_.setTransportPendingMove(guid, x, y, z, orientation);
                    LOG_DEBUG("Cannot auto-spawn transport 0x", std::hex, guid, std::dec,
                              " - WMO instance not found yet (queued move for replay)");
                    return;
                }
            } else {
                entitySpawner_.setTransportPendingMove(guid, x, y, z, orientation);
                LOG_DEBUG("Cannot auto-spawn transport 0x", std::hex, guid, std::dec,
                          " - entity not found in EntityManager (queued move for replay)");
                return;
            }
        }

        // Update TransportManager's internal state (position, rotation, transform matrices)
        // This also updates the WMO renderer automatically
        // Coordinates are already canonical (converted in game_handler.cpp when entity was created)
        glm::vec3 canonicalPos(x, y, z);
        transportManager->updateServerTransport(guid, canonicalPos, orientation);

        // Move player with transport if riding it
        if (gameHandler_.isOnTransport() && gameHandler_.getPlayerTransportGuid() == guid) {
            auto* cc = renderer_.getCameraController();
            if (cc) {
                glm::vec3* ft = cc->getFollowTargetMutable();
                if (ft) {
                    // Get player world position from TransportManager (handles transform properly)
                    glm::vec3 offset = gameHandler_.getPlayerTransportOffset();
                    glm::vec3 worldPos = transportManager->getPlayerWorldPosition(guid, offset);
                    *ft = worldPos;
                }
            }
        }
    });
}

}} // namespace wowee::core
