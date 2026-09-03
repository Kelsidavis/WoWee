#include "ui/scene_pick.hpp"

#include "core/application.hpp"
#include "core/coordinates.hpp"
#include "game/game_handler.hpp"
#include "game/entity.hpp"
#include "rendering/camera.hpp"

#include <algorithm>
#include <cmath>

namespace wowee {
namespace ui {

namespace {

/// A wide forge or anvil would otherwise swallow the NPCs standing at it.
constexpr float kMaxGameObjectPickRadius = 3.0f;
/// ...and a mug is still worth a click at arm's length. Small enough that
/// scenery beside the player does not stand in front of what is being aimed
/// at, large enough that a small object is not a pixel hunt.
constexpr float kMinGameObjectPickRadius = 0.6f;
/// GAMEOBJECT_TYPE_CHAIR.
constexpr uint32_t kGoTypeChair = 7;
/// GENERIC - decorative, no interaction - and FISHINGHOLE, which is fished
/// rather than clicked. Neither may take a click from a unit.
constexpr uint32_t kGoTypeGeneric = 5;
constexpr uint32_t kGoTypeFishingHole = 25;

bool isDeadUnit(const std::shared_ptr<game::Entity>& entity) {
    auto unit = std::dynamic_pointer_cast<game::Unit>(entity);
    return unit && unit->getHealth() == 0 && unit->getMaxHealth() > 0;
}

} // namespace

ScenePick pickScene(game::GameHandler& gameHandler,
                    const rendering::Ray& ray,
                    const ScenePickParams& params,
                    const ScenePickHit& onHit) {
    ScenePick pick;
    const uint64_t myGuid = gameHandler.getPlayerGuid();

    for (const auto& [guid, entity] : gameHandler.getEntityManager().getEntities()) {
        if (!entity) continue;
        const auto type = entity->getType();
        if (type != game::ObjectType::UNIT &&
            type != game::ObjectType::PLAYER &&
            type != game::ObjectType::GAMEOBJECT) continue;
        if (guid == myGuid) continue;  // never target yourself

        // Nor the scenery. A trigger is an ordinary unit on the wire - it has
        // health, a name and a model - and UNIT_FLAG_NOT_SELECTABLE is the
        // server saying it is not there to be clicked.
        if (type == game::ObjectType::UNIT) {
            auto unit = std::static_pointer_cast<game::Unit>(entity);
            if (unit->getUnitFlags() & game::UNIT_FLAG_NOT_SELECTABLE) continue;
        }

        const game::GameObjectQueryResponseData* goInfo = nullptr;
        if (type == game::ObjectType::GAMEOBJECT) {
            auto go = std::static_pointer_cast<game::GameObject>(entity);
            goInfo = gameHandler.getCachedGameObjectInfo(go->getEntry());
            if (params.skipChairs && goInfo && goInfo->type == kGoTypeChair) continue;
        }

        glm::vec3 hitCenter;
        float hitRadius = 0.0f;
        const bool hasBounds =
            core::Application::getInstance().getRenderBoundsForGuid(guid, hitCenter, hitRadius);
        if (!hasBounds) {
            // Match the hover-cursor sizes, so the reticle agrees with the
            // cursor affordance - otherwise NPCs feel hard to click.
            float heightOffset = params.unitHeightOffset;
            hitRadius = params.unitHitRadius;
            if (type == game::ObjectType::UNIT) {
                auto unit = std::static_pointer_cast<game::Unit>(entity);
                // Critters have very low max health.
                if (unit->getMaxHealth() > 0 && unit->getMaxHealth() < 100) {
                    hitRadius = 0.5f;
                    heightOffset = 0.3f;
                }
            } else if (type == game::ObjectType::GAMEOBJECT) {
                hitRadius = 2.5f;
                heightOffset = 1.2f;
            }
            hitCenter = core::coords::canonicalToRender(
                glm::vec3(entity->getX(), entity->getY(), entity->getZ()));
            hitCenter.z += heightOffset;
        } else if (type == game::ObjectType::GAMEOBJECT) {
            // An object's own size, a little forgiving - not a minimum of two
            // and a half yards.
            //
            // Every object was inflated to that floor whatever it was, so a mug
            // on a table and a chair beside the player wore the same sphere a
            // forge does. Standing near small scenery, the ray entered its
            // sphere on the way to whatever was actually being pointed at, and
            // the near thing won for being near: the click landed on the chair
            // rather than the door across the room.
            hitRadius = std::clamp(hitRadius * 1.15f, kMinGameObjectPickRadius,
                                   kMaxGameObjectPickRadius);
        } else {
            hitRadius = std::max(hitRadius * 1.25f, 1.0f);
        }

        float hitT = 0.0f;
        if (!raySphereIntersect(ray, hitCenter, hitRadius, hitT)) continue;
        const float centerT = glm::dot(hitCenter - ray.origin, ray.direction);

        if (type == game::ObjectType::UNIT || type == game::ObjectType::PLAYER) {
            // Rank the living ahead of the dead rather than by distance alone. A
            // corpse lies at ground level while a unit's sphere sits a metre up,
            // so looking down at someone standing on a body put the body nearer.
            if (isDeadUnit(entity)) {
                if (centerT < pick.deadUnitCenterT) {
                    pick.deadUnitCenterT = centerT;
                    pick.deadUnitGuid = guid;
                }
            } else {
                if (centerT < pick.livingUnitCenterT) {
                    pick.livingUnitCenterT = centerT;
                    pick.livingUnitGuid = guid;
                }
                if (type == game::ObjectType::UNIT) {
                    auto unit = std::static_pointer_cast<game::Unit>(entity);
                    // A corpse still answers isHostile(), and a hostile is picked
                    // ahead of everything - hence the liveness check above it.
                    const bool hostile = unit->isHostile() ||
                                         gameHandler.isAggressiveTowardPlayer(guid);
                    if (hostile && hitT < pick.hostileUnitT) {
                        pick.hostileUnitT = hitT;
                        pick.hostileUnitGuid = guid;
                    }
                }
            }
        } else if (type == game::ObjectType::GAMEOBJECT) {
            const bool interactive = !goInfo || (goInfo->type != kGoTypeGeneric &&
                                                 goInfo->type != kGoTypeFishingHole);
            // How well the ray is aimed at this one, as a fraction of its own
            // size: nought is dead centre and one is a graze. Ranked by that
            // rather than by which centre is nearest along the ray, because
            // "nearest" is the question that made scenery at the player's elbow
            // beat the thing they were pointing at - a ray that grazes the edge
            // of a near sphere is still a nearer hit than one straight through
            // the middle of a far one.
            const float centreOffset = glm::length(hitCenter - (ray.origin + ray.direction * centerT));
            const float aim = hitRadius > 0.0f ? centreOffset / hitRadius : 1.0f;
            if (interactive && aim < pick.objectAim) {
                pick.objectAim = aim;
                pick.objectCenterT = centerT;
                pick.objectGuid = guid;
            }
        }

        if (hitT < pick.closestT) {
            pick.closestT = hitT;
            pick.closestGuid = guid;
        }

        if (onHit) onHit(guid, entity, hitT, centerT);
    }

    return pick;
}

} // namespace ui
} // namespace wowee
