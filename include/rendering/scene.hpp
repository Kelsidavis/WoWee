#pragma once

#include <vector>
#include <memory>

namespace wowee {
namespace rendering {

class Mesh;

class Scene {
public:
    Scene() = default;
    ~Scene() = default;

    void addMesh(std::shared_ptr<Mesh> mesh);
    void removeMesh(const std::shared_ptr<Mesh>& mesh);
    void clear();

    const std::vector<std::shared_ptr<Mesh>>& getMeshes() const { return meshes; }

private:
    std::vector<std::shared_ptr<Mesh>> meshes;
};

} // namespace rendering
} // namespace wowee
