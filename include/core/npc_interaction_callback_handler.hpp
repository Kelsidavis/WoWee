#pragma once

#include <cstdint>
#include <glm/glm.hpp>
#include "audio/npc_voice_manager.hpp"

namespace wowee {

namespace rendering { class Renderer; }
namespace game { class GameHandler; }
namespace audio { class AudioCoordinator; }
namespace core { class EntitySpawner; }

namespace core {

/// Handles NPC interaction callbacks: greeting, farewell, vendor, aggro voice lines.
class NPCInteractionCallbackHandler {
public:
    NPCInteractionCallbackHandler(EntitySpawner& entitySpawner,
                                  rendering::Renderer* renderer,
                                  game::GameHandler& gameHandler,
                                  audio::AudioCoordinator* audioCoordinator);

    void setupCallbacks();

private:
    /// Resolve NPC voice type from GUID (eliminates 4x copy-paste of display-ID lookup)
    [[nodiscard]] audio::VoiceType resolveNpcVoiceType(uint64_t guid) const;

    EntitySpawner& entitySpawner_;
    rendering::Renderer* renderer_;
    game::GameHandler& gameHandler_;
    audio::AudioCoordinator* audioCoordinator_;
};

} // namespace core
} // namespace wowee
