#pragma once

// Renamed from PlayerAnimator/NpcAnimator dual-map → unified CharacterAnimator registry.
// NpcAnimator removed - all characters use the same generic CharacterAnimator.
#include "rendering/animation/character_animator.hpp"
#include "rendering/animation/anim_capability_set.hpp"
#include <cstdint>
#include <unordered_map>
#include <memory>

namespace wowee {
namespace rendering {

// ============================================================================
// AnimationManager
//
// Central registry for all character animators. Owned by Renderer, replaces
// scattered AnimationController* passing.
//
// Single animator type:
//   CharacterAnimator - generic animator for any character (player, NPC,
//                        companion). Full FSM composition with priority
//                        resolver.
//
// AnimationController becomes a thin shim delegating to this manager
// until all callsites are migrated.
// ============================================================================
class AnimationManager {
public:
    AnimationManager() = default;

    // ── Character animators ─────────────────────────────────────────────


    /// Get existing CharacterAnimator (nullptr if not found).
    CharacterAnimator* get(uint32_t instanceId);

    /// Remove a character animator.
    void remove(uint32_t instanceId);

    // ── Per-frame ───────────────────────────────────────────────────────


    // ── Counts ──────────────────────────────────────────────────────────
    [[nodiscard]] size_t count() const { return animators_.size(); }

private:
    std::unordered_map<uint32_t, std::unique_ptr<CharacterAnimator>> animators_;
};

} // namespace rendering
} // namespace wowee
