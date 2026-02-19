#include "game/entity.hpp"
#include "core/logger.hpp"

namespace wowee {
namespace game {

void EntityManager::addEntity(uint64_t guid, std::shared_ptr<Entity> entity) {
    if (!entity) {
        LOG_WARNING("Attempted to add null entity with GUID: 0x", std::hex, guid, std::dec);
        return;
    }

    entities[guid] = std::move(entity);

    LOG_DEBUG("Added entity: GUID=0x", std::hex, guid, std::dec,
              ", Type=", static_cast<int>(entity->getType()));
}

void EntityManager::removeEntity(uint64_t guid) {
    auto it = entities.find(guid);
    if (it != entities.end()) {
        LOG_DEBUG("Removed entity: GUID=0x", std::hex, guid, std::dec);
        entities.erase(it);
    }
}

std::shared_ptr<Entity> EntityManager::getEntity(uint64_t guid) const {
    auto it = entities.find(guid);
    return (it != entities.end()) ? it->second : nullptr;
}

bool EntityManager::hasEntity(uint64_t guid) const {
    return entities.find(guid) != entities.end();
}

} // namespace game
} // namespace wowee
