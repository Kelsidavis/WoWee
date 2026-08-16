#pragma once

#include <memory>
#include <utility>
#include <glm/glm.hpp>

namespace wowee {
namespace rendering {

class Shader;
class Texture;

class Material {
public:
    Material() = default;
    ~Material() = default;

    void setShader(std::shared_ptr<Shader> shader) { this->shader = std::move(shader); }
    void setTexture(std::shared_ptr<Texture> texture) { this->texture = std::move(texture); }
    void setColor(const glm::vec4& color) { this->color = color; }

    [[nodiscard]] std::shared_ptr<Shader> getShader() const { return shader; }
    [[nodiscard]] std::shared_ptr<Texture> getTexture() const { return texture; }
    [[nodiscard]] const glm::vec4& getColor() const { return color; }

private:
    std::shared_ptr<Shader> shader;
    std::shared_ptr<Texture> texture;
    glm::vec4 color = glm::vec4(1.0f);
};

} // namespace rendering
} // namespace wowee
