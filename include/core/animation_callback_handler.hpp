#pragma once

#include <cstdint>
#include <glm/glm.hpp>

namespace wowee {

namespace rendering { class Renderer; }
namespace game { class GameHandler; }
namespace core { class AppearanceComposer; class EntitySpawner; }

namespace core {

/// Handles animation callbacks: death, respawn, swing, hit reaction, spell cast, emote,
/// stun, stealth, health, ghost, stand state, loot, sprint, vehicle, charge.
/// Owns charge rush state (interpolated in update).
class AnimationCallbackHandler {
public:
    AnimationCallbackHandler(EntitySpawner& entitySpawner,
                             rendering::Renderer& renderer,
                             game::GameHandler& gameHandler,
                             AppearanceComposer& appearanceComposer);

    void setupCallbacks();

    /// Called each frame from Application::update() to drive charge interpolation.
    /// Returns true if charge is active (player is externally driven).
    bool updateCharge(float deltaTime);

    // Charge state queries (used by Application::update for externallyDrivenMotion)
    [[nodiscard]] bool isCharging() const { return chargeActive_; }

    // Reset charge state (logout/disconnect)
    void resetChargeState();

private:
    EntitySpawner& entitySpawner_;
    rendering::Renderer& renderer_;
    game::GameHandler& gameHandler_;
    AppearanceComposer& appearanceComposer_;

    // Charge rush state (moved from Application)
    bool chargeActive_ = false;
    float chargeTimer_ = 0.0f;
    float chargeDuration_ = 0.0f;
    glm::vec3 chargeStartPos_{0.0f};  // Render coordinates
    glm::vec3 chargeEndPos_{0.0f};    // Render coordinates
    uint64_t chargeTargetGuid_ = 0;
};

} // namespace core
} // namespace wowee
