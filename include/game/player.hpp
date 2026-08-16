#pragma once

#include <glm/glm.hpp>

namespace wowee {
namespace game {

class Player {
public:
    void setPosition(const glm::vec3& pos) { position = pos; }
    [[nodiscard]] const glm::vec3& getPosition() const { return position; }

private:
    glm::vec3 position = glm::vec3(0.0f);
};

} // namespace game
} // namespace wowee
