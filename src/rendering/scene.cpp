#include "rendering/scene.hpp"
#include "rendering/mesh.hpp"
#include <algorithm>

namespace wowee {
namespace rendering {

void Scene::addMesh(std::shared_ptr<Mesh> mesh) {
    meshes.push_back(std::move(mesh));
}

void Scene::removeMesh(const std::shared_ptr<Mesh>& mesh) {
    auto it = std::find(meshes.begin(), meshes.end(), mesh);
    if (it != meshes.end()) {
        meshes.erase(it);
    }
}

void Scene::clear() {
    meshes.clear();
}

} // namespace rendering
} // namespace wowee
