#pragma once

#include <cstdint>
#include <optional>
#include <glm/glm.hpp>

namespace wowee {

namespace rendering { class Renderer; }
namespace game { class GameHandler; }
namespace pipeline { class AssetManager; }
namespace audio { class AudioCoordinator; }
namespace core { class EntitySpawner; class WorldLoader; }

namespace core {

/// Handles world entry, unstuck, hearthstone, and bind point callbacks.
/// Owns hearth-teleport state and worldEntryMovementGraceTimer.
class WorldEntryCallbackHandler {
public:
    WorldEntryCallbackHandler(rendering::Renderer& renderer,
                              game::GameHandler& gameHandler,
                              WorldLoader* worldLoader,
                              EntitySpawner* entitySpawner,
                              pipeline::AssetManager* assetManager);

    void setupCallbacks();

    /// Called each frame from Application::update() to manage hearth-teleport freeze/thaw.
    void update(float deltaTime);

    // State queries (used by Application::update)
    [[nodiscard]] float getWorldEntryMovementGraceTimer() const { return worldEntryMovementGraceTimer_; }
    void setWorldEntryMovementGraceTimer(float t) { worldEntryMovementGraceTimer_ = t; }
    [[nodiscard]] bool isHearthTeleportPending() const { return hearthTeleportPending_; }

    // Reset state (logout/disconnect)
    void resetState();

    // Taxi state (managed by Application::update, but tracked here for clarity)
    [[nodiscard]] bool getLastTaxiFlight() const { return lastTaxiFlight_; }
    void setLastTaxiFlight(bool v) { lastTaxiFlight_ = v; }
    [[nodiscard]] float getTaxiLandingClampTimer() const { return taxiLandingClampTimer_; }
    void setTaxiLandingClampTimer(float t) { taxiLandingClampTimer_ = t; }
    // Character render-position Z captured the moment the landing clamp arms,
    // i.e. wherever the taxi flight itself left the player. Used to pick whichever
    // floor candidate (terrain/WMO/M2) is closest to the real landing point instead
    // of unconditionally preferring WMO/M2 over terrain (see application.cpp).
    [[nodiscard]] float getTaxiLandingReferenceZ() const { return taxiLandingReferenceZ_; }
    void setTaxiLandingReferenceZ(float z) { taxiLandingReferenceZ_ = z; }

private:
    /// Sample best floor height at (x, y) from terrain, WMO, and M2 (eliminates 3x duplication)
    [[nodiscard]] std::optional<float> sampleBestFloorAt(float x, float y, float probeZ) const;

    /// Clear stuck movement state on player
    void clearStuckMovement();

    /// Dismount and remove the local mount before an unstuck teleport.
    void clearMountForUnstuck();

    /// Sync teleported render position to server
    void syncTeleportedPositionToServer(const glm::vec3& renderPos);

    /// Force server-side teleport via GM command
    void forceServerTeleportCommand(const glm::vec3& renderPos);

    rendering::Renderer& renderer_;
    game::GameHandler& gameHandler_;
    WorldLoader* worldLoader_;
    EntitySpawner* entitySpawner_;
    pipeline::AssetManager* assetManager_;

    // Hearth teleport: freeze player until terrain loads at destination (moved from Application)
    bool hearthTeleportPending_ = false;
    glm::vec3 hearthTeleportPos_{0.0f};  // render coords
    float hearthTeleportTimer_ = 0.0f;   // timeout safety

    float worldEntryMovementGraceTimer_ = 0.0f;
    bool lastTaxiFlight_ = false;
    float taxiLandingClampTimer_ = 0.0f;
    float taxiLandingReferenceZ_ = 0.0f;
};

} // namespace core
} // namespace wowee
