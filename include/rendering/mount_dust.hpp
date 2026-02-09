#pragma once

#include <GL/glew.h>
#include <glm/glm.hpp>
#include <memory>
#include <vector>

namespace wowee {
namespace rendering {

class Camera;
class Shader;

class MountDust {
public:
    MountDust();
    ~MountDust();

    bool initialize();
    void shutdown();

    // Spawn dust particles at mount feet when moving on ground
    void spawnDust(const glm::vec3& position, const glm::vec3& velocity, bool isMoving);

    void update(float deltaTime);
    void render(const Camera& camera);

private:
    struct Particle {
        glm::vec3 position;
        glm::vec3 velocity;
        float lifetime;
        float maxLifetime;
        float size;
        float alpha;
    };

    static constexpr int MAX_DUST_PARTICLES = 300;
    std::vector<Particle> particles;

    GLuint vao = 0;
    GLuint vbo = 0;
    std::unique_ptr<Shader> shader;
    std::vector<float> vertexData;

    float spawnAccum = 0.0f;
};

} // namespace rendering
} // namespace wowee
