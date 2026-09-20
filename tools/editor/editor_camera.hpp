#pragma once

#include "rendering/camera.hpp"
#include <SDL3/SDL.h>
#include <glm/glm.hpp>
#include <cmath>

namespace wowee {
namespace editor {

class EditorCamera {
public:
    EditorCamera();

    void update(float deltaTime);
    void processMouseMotion(int dx, int dy);
    void processMouseWheel(float delta, bool shiftHeld);
    void processKeyEvent(const SDL_KeyboardEvent& event);
    void processMouseButton(const SDL_MouseButtonEvent& event);
    void processMiddleMouseMotion(int dx, int dy, const glm::vec3& pivotPoint);

    rendering::Camera& getCamera() { return camera_; }
    const rendering::Camera& getCamera() const { return camera_; }

    float getSpeed() const { return speed_; }
    void setSpeed(float s) {
        // Match the wheel-zoom clamp range. NaN/inf would propagate
        // into camera position via update(); the cap also matches what
        // the user can set via the wheel UI.
        if (std::isfinite(s) && s >= 10.0f && s <= 2000.0f) speed_ = s;
    }
    void setPosition(const glm::vec3& pos);
    void setYawPitch(float yaw, float pitch);

private:
    rendering::Camera camera_;
    float speed_ = 100.0f;
    float yaw_ = 0.0f;
    float pitch_ = 0.0f;
    bool keyW_ = false, keyA_ = false, keyS_ = false, keyD_ = false;
    bool keyQ_ = false, keyE_ = false, keyShift_ = false;
    bool rightMouseDown_ = false;
    bool middleMouseDown_ = false;
};

} // namespace editor
} // namespace wowee
