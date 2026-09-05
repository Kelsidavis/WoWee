#pragma once

#include "game/expansion_profile.hpp"
#include "game/item_text.hpp"
#include "core/application.hpp"

#include <cstdint>

namespace wowee {
namespace game {

inline bool isActiveExpansion(const char* expansionId) {
    auto& app = core::Application::getInstance();
    auto* registry = app.getExpansionRegistry();
    if (!registry) return false;
    auto* profile = registry->getActive();
    if (!profile) return false;
    return profile->id == expansionId;
}

inline bool isClassicLikeExpansion() {
    return isActiveExpansion("classic") || isActiveExpansion("turtle");
}

inline bool isPreWotlk() {
    return isClassicLikeExpansion() || isActiveExpansion("tbc");
}

// Shared item link formatter used by inventory, quest, spell, and social
// handlers. It is a second name for itemChatLink and not a second copy: this
// one carried its own quality table, which is how the colours came to be
// written out in six places for one set of eight values.
inline std::string buildItemLink(uint32_t itemId, uint32_t quality, const std::string& name) {
    return itemChatLink(itemId, quality, name);
}

/// Whether a map 0 position is the corrupted one rather than real ground.
///
/// A faulty area-trigger destination leaves the server holding a position at
/// the origin, and it persists across sessions: on re-login it arrives in
/// LOGIN_VERIFY_WORLD, and heartbeats sent from there write it back. Several
/// paths refuse it - the teleport ack, the player's own update blocks, the
/// heartbeat, the area-trigger sweep - and all of them ask this.
///
/// The test used to be a thousand units square, which is not "the origin" on
/// map 0: Hillsbrad sits inside it. Standing in Southshore, every heartbeat
/// was blocked, so the server went on believing the player was wherever they
/// had been before - and streamed that place's creatures instead. An empty
/// Southshore was this box, not a missing spawn.
///
/// Ten units on each axis, height included. A zeroed position matches; ground
/// a player is standing on does not, the origin's own terrain being some way
/// below or above z = 0 wherever the two coincide.
constexpr bool isCorruptOriginPosition(uint32_t mapId, float x, float y, float z) {
    constexpr float kNearOrigin = 10.0f;
    const auto near = [](float v) { return v < kNearOrigin && v > -kNearOrigin; };
    return mapId == 0 && near(x) && near(y) && near(z);
}

} // namespace game
} // namespace wowee
