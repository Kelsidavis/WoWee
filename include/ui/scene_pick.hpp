#pragma once

// Ray-picking entities under the cursor, shared by the two callers that need it.
//
// The left-click target picker and the right-click world picker each grew their
// own copy of this traversal. They agreed on everything that matters - how a
// unit is ranked against an object, that a hostile is preferred, that a corpse
// must not outrank the living - and had to be edited in lockstep to stay that
// way, which is exactly how one of them quietly diverges. What genuinely
// differs between them is expressed as parameters instead.

#include <cstdint>
#include <functional>
#include <memory>

#include <cmath>

#include <glm/glm.hpp>

#include "rendering/camera.hpp"  // rendering::Ray

namespace wowee {
namespace game { class GameHandler; class Entity; }

namespace ui {

/// Where a ray first meets a sphere, if it does at all.
///
/// The picker's own test, and the one every hand-rolled click target in the
/// interface needs. It was written out five times - once here and once in each
/// of the four files GameScreen was split into - and three of those four copies
/// were never called from the file they sat in. Identical to the character, so
/// nothing had gone wrong yet; five copies of a quadratic is simply five
/// chances for one of them to lose a sign.
inline bool raySphereIntersect(const rendering::Ray& ray, const glm::vec3& center,
                               float radius, float& tOut) {
    const glm::vec3 oc = ray.origin - center;
    const float b = glm::dot(oc, ray.direction);
    const float c = glm::dot(oc, oc) - radius * radius;
    const float disc = b * b - c;
    if (disc < 0.0f) return false;
    const float root = std::sqrt(disc);
    float t = -b - root;
    // Behind the near root means the origin is inside the sphere; the far root
    // is then the first surface the ray meets going forward.
    if (t < 0.0f) t = -b + root;
    if (t < 0.0f) return false;
    tOut = t;
    return true;
}

/// The parts of picking the two callers disagree about.
struct ScenePickParams {
    /// Fallback hit sphere for a unit with no render bounds. The right-click
    /// picker uses a slightly taller, tighter sphere than the target picker.
    float unitHitRadius = 1.8f;
    float unitHeightOffset = 1.0f;
    /// Skip chair-type game objects. Their fallback sphere is wide enough to be
    /// caught while right-drag-rotating the camera near one, which sits the
    /// player down. Left-clicking a chair to target it still works.
    bool skipChairs = false;
};

/// Everything the traversal found, before either caller applies its own rules.
struct ScenePick {
    uint64_t closestGuid = 0;          ///< Nearest hit of any pickable type.
    float    closestT = 1e30f;

    uint64_t hostileUnitGuid = 0;      ///< Nearest *living* hostile unit.
    float    hostileUnitT = 1e30f;

    uint64_t livingUnitGuid = 0;       ///< Nearest living unit or player, by centre.
    float    livingUnitCenterT = 1e30f;
    uint64_t deadUnitGuid = 0;         ///< Nearest corpse, by centre.
    float    deadUnitCenterT = 1e30f;

    uint64_t objectGuid = 0;           ///< Nearest interactable game object, by centre.
    float    objectCenterT = 1e30f;

    /// The living win; a corpse is still selectable, but only when nothing alive
    /// was under the cursor - which is what makes a player standing on a body
    /// clickable, and what looting and skinning still need.
    [[nodiscard]] uint64_t unitGuid() const { return livingUnitGuid != 0 ? livingUnitGuid : deadUnitGuid; }
    [[nodiscard]] float unitCenterT() const {
        return livingUnitGuid != 0 ? livingUnitCenterT : deadUnitCenterT;
    }

    /// Resolve to the one thing under the cursor: a unit unless an object's
    /// centre is clearly in front of it, so a creature is never lost to a
    /// decorative or backing object behind it. A hostile keeps its priority.
    ///
    /// This and not closestGuid is what a caller wants. closestGuid is the
    /// nearest *entry point* of anything pickable, and a game object's sphere
    /// is 2.5 yards against a unit's 1.8 - so a wide object beside an NPC is
    /// entered first even when the NPC is nearer, and answers closestGuid while
    /// a click still lands on the NPC. kMaxGameObjectPickRadius exists for that
    /// same reason. Every affordance drawn for what a click would do has to ask
    /// this one.
    ///
    /// Inline so the rule can be tested without linking the picker's traversal,
    /// which needs the whole client behind it.
    [[nodiscard]] uint64_t resolve() const {
        // A unit wins over a game object unless the object's centre is clearly
        // in front of the unit's.
        constexpr float kUnitOverGoBias = 2.0f;
        const uint64_t unit = unitGuid();
        if (unit != 0 && (objectGuid == 0 || unitCenterT() <= objectCenterT + kUnitOverGoBias)) {
            return hostileUnitGuid != 0 ? hostileUnitGuid : unit;
        }
        if (objectGuid != 0) return objectGuid;
        return 0;
    }
};

/// Called for every entity the ray hits, so a caller can gather what only it
/// cares about (a hooked fishing bobber, quest objective objects) without
/// repeating the traversal.
using ScenePickHit = std::function<void(uint64_t guid,
                                        const std::shared_ptr<game::Entity>& entity,
                                        float hitT, float centerT)>;

ScenePick pickScene(game::GameHandler& gameHandler,
                    const rendering::Ray& ray,
                    const ScenePickParams& params,
                    const ScenePickHit& onHit = {});

} // namespace ui
} // namespace wowee
